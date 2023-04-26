/*!
 * @file
 * @brief This file contains implementation of gpu
 *
 * @author Tomáš Milet, imilet@fit.vutbr.cz
 */

#include <student/gpu.hpp>



void clear(GPUMemory& mem, ClearCommand cmd) {
    if (cmd.clearColor) {
        float red = cmd.color.r;
        float green = cmd.color.g;
        float blue = cmd.color.b;
        float alpha = cmd.color.a;
        mem.framebuffer.color = new uint8_t[mem.framebuffer.width * mem.framebuffer.height * mem.framebuffer.channels];
        for (uint32_t i = 0; i < mem.framebuffer.width * mem.framebuffer.height; ++i) {
            mem.framebuffer.color[i * mem.framebuffer.channels + 0] = static_cast<uint8_t>(red * 255.f);
            mem.framebuffer.color[i * mem.framebuffer.channels + 1] = static_cast<uint8_t>(green * 255.f);
            mem.framebuffer.color[i * mem.framebuffer.channels + 2] = static_cast<uint8_t>(blue * 255.f);
            mem.framebuffer.color[i * mem.framebuffer.channels + 3] = static_cast<uint8_t>(alpha * 255.f);
        }
    }
    if (cmd.clearDepth) {
        float* depthBuffer = new float[mem.framebuffer.width * mem.framebuffer.height];
        for (uint32_t i = 0; i < mem.framebuffer.width * mem.framebuffer.height; ++i) {
            depthBuffer[i] = cmd.depth;
        }
        mem.framebuffer.depth = depthBuffer;
    }
}




void computeVertexID(InVertex& inVertex, uint32_t vertexID){
  inVertex.gl_VertexID = vertexID;
}

void readAttributes(InVertex& inVertex, VertexArray& vao, bool indexed, GPUMemory* mem, uint32_t vertexID){
  for (int j = 0; j < maxAttributes; j++) {
    if (vao.vertexAttrib[j].type == AttributeType::EMPTY) {
      continue;
    }
    int32_t bufferID = vao.vertexAttrib[j].bufferID;
    uint64_t stride = vao.vertexAttrib[j].stride;
    uint64_t offset = vao.vertexAttrib[j].offset;

    switch (vao.vertexAttrib[j].type) {
      case AttributeType::FLOAT:
        inVertex.attributes[j].v1 = *(float*)(mem->buffers[bufferID].data + offset + vertexID * stride);
        break;
      case AttributeType::VEC2:
        inVertex.attributes[j].v2 = *(glm::vec2*)(mem->buffers[bufferID].data + offset + vertexID * stride);
        break;
      case AttributeType::VEC3:
        inVertex.attributes[j].v3 = *(glm::vec3*)(mem->buffers[bufferID].data + offset + vertexID * stride);
        break;
      case AttributeType::VEC4:
        inVertex.attributes[j].v4 = *(glm::vec4*)(mem->buffers[bufferID].data + offset + vertexID * stride);
        break;
    }
  }
}

void runVertexAssembly(InVertex& inVertex, VertexArray& vao, bool indexed, GPUMemory& mem, uint32_t vertexID, uint32_t currentCommand){
  if (indexed) {
    uint64_t indexOffset = vao.indexOffset + (vertexID * static_cast<uint64_t>(vao.indexType));
    uint32_t index = 0;
    switch (vao.indexType) {
        case IndexType::UINT8:
            index = *((uint8_t*)(mem.buffers[vao.indexBufferID].data + indexOffset));
            break;
        case IndexType::UINT16:
            index = *((uint16_t*)(mem.buffers[vao.indexBufferID].data + indexOffset));
            break;
        case IndexType::UINT32:
            index = *((uint32_t*)(mem.buffers[vao.indexBufferID].data + indexOffset));
            break;
    }
    inVertex.gl_VertexID = index;
  } else {
    inVertex.gl_VertexID = vertexID;
  }
  inVertex.gl_DrawID = currentCommand;
  readAttributes(inVertex, vao, indexed, &mem, vertexID);
}



void draw(GPUMemory& mem, DrawCommand cmd, uint32_t currentCommand) {
    Program prg = mem.programs[cmd.programID];
    VertexArray vao = cmd.vao;
    bool indexed = vao.indexBufferID != -1;

    for (uint32_t i = 0; i < cmd.nofVertices; ++i) {
        InVertex inVertex;
        OutVertex outVertex;
        if (indexed) {
            uint64_t indexOffset = vao.indexOffset + (i * static_cast<uint64_t>(vao.indexType));
            uint32_t index = 0;
            switch (vao.indexType) {
                case IndexType::UINT8:
                    index = *((uint8_t*)(mem.buffers[vao.indexBufferID].data + indexOffset));
                    break;
                case IndexType::UINT16:
                    index = *((uint16_t*)(mem.buffers[vao.indexBufferID].data + indexOffset));
                    break;
                case IndexType::UINT32:
                    index = *((uint32_t*)(mem.buffers[vao.indexBufferID].data + indexOffset));
                    break;
            }
            inVertex.gl_VertexID = index;
        } else {
            inVertex.gl_VertexID = i;
        }
        inVertex.gl_DrawID = currentCommand;

        // Read the attributes for this vertex from the VAO
        for (int j = 0; j < maxAttributes; j++) {
            if (vao.vertexAttrib[j].type == AttributeType::EMPTY) {
                continue;
            }
            int32_t bufferID = vao.vertexAttrib[j].bufferID;
            uint64_t stride = vao.vertexAttrib[j].stride;
            uint64_t offset = vao.vertexAttrib[j].offset;

          switch (vao.vertexAttrib[j].type) {
                  case AttributeType::FLOAT:
                      inVertex.attributes[j].v1 = *(float*)(mem.buffers[bufferID].data + offset + i * stride);
                      break;
                  case AttributeType::VEC2:
                      inVertex.attributes[j].v2 = *(glm::vec2*)(mem.buffers[bufferID].data + offset + i * stride);
                      break;
                  case AttributeType::VEC3:
                      inVertex.attributes[j].v3 = *(glm::vec3*)(mem.buffers[bufferID].data + offset + i * stride);
                      break;
                  case AttributeType::VEC4:
                      inVertex.attributes[j].v4 = *(glm::vec4*)(mem.buffers[bufferID].data + offset + i * stride);
                      break;
              }
        }

        runVertexAssembly(inVertex, vao, indexed, mem, i, currentCommand);
        ShaderInterface si;
        prg.vertexShader(outVertex, inVertex, si);
    }
}




//! [gpu_execute]
void gpu_execute(GPUMemory&mem,CommandBuffer &cb){
    uint32_t currentCommand = 0;
    for(uint32_t i=0;i<cb.nofCommands;++i){
        CommandType type = cb.commands[i].type;
        CommandData data = cb.commands[i].data;
        if(type == CommandType::CLEAR){
            clear(mem,data.clearCommand);
        }
        if(type == CommandType::DRAW){
            draw(mem, data.drawCommand, currentCommand);
            ++currentCommand;
        }
    }
}
//! [gpu_execute]




//      if(type == CommandType::DRAW){draw(mem,data.drawCommand);}

/**
 * @brief This function reads color from texture.
 *
 * @param texture texture
 * @param uv uv coordinates
 *
 * @return color 4 floats
 */
glm::vec4 read_texture(Texture const&texture,glm::vec2 uv){
  if(!texture.data)return glm::vec4(0.f);
  auto uv1 = glm::fract(uv);
  auto uv2 = uv1*glm::vec2(texture.width-1,texture.height-1)+0.5f;
  auto pix = glm::uvec2(uv2);
  //auto t   = glm::fract(uv2);
  glm::vec4 color = glm::vec4(0.f,0.f,0.f,1.f);
  for(uint32_t c=0;c<texture.channels;++c)
    color[c] = texture.data[(pix.y*texture.width+pix.x)*texture.channels+c]/255.f;
  return color;
}

