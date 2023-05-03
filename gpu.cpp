/*!
 * @file
 * @brief This file contains implementation of gpu
 *
 * @author Tomáš Milet, imilet@fit.vutbr.cz
 */

#include <student/gpu.hpp>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>


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
    mem.framebuffer.depth = new float[mem.framebuffer.width * mem.framebuffer.height];
        for (uint32_t i = 0; i < mem.framebuffer.width * mem.framebuffer.height; ++i) {
            mem.framebuffer.depth[i] = cmd.depth;
        }
    }
}




void rasterizeTriangle(GPUMemory& mem, OutVertex* vertices, uint32_t primitiveID, DrawCommand const& drawCmd) {
    bool backFaceCulling = drawCmd.backfaceCulling;
    glm::vec3 positions[3];
    for (int i = 0; i < 3; ++i) {
        positions[i] = glm::vec3(vertices[i].gl_Position) / vertices[i].gl_Position.w;
    }

    glm::vec4 viewport(0, 0, mem.framebuffer.width, mem.framebuffer.height);
    for (int i = 0; i < 3; ++i) {
        positions[i] = (positions[i] + 1.0f) * 0.5f * glm::vec3(viewport.z, viewport.w, 1.0f);
    }

    // Implement triangle rasterization and fragment shader invocation here
    glm::ivec2 minCoords(
        std::max(0, static_cast<int>(std::floor(std::min(positions[0].x, std::min(positions[1].x, positions[2].x))))),
        std::max(0, static_cast<int>(std::floor(std::min(positions[0].y, std::min(positions[1].y, positions[2].y))))));

    glm::ivec2 maxCoords(
        std::min(static_cast<int>(mem.framebuffer.width) - 1, static_cast<int>(std::ceil(std::max(positions[0].x, std::max(positions[1].x, positions[2].x))))),
        std::min(static_cast<int>(mem.framebuffer.height) - 1, static_cast<int>(std::ceil(std::max(positions[0].y, std::max(positions[1].y, positions[2].y))))));

    ShaderInterface tempShaderInterface;
    tempShaderInterface.uniforms = &mem.uniforms[0];


      glm::vec3 edge1 = positions[1] - positions[0];
      glm::vec3 edge2 = positions[2] - positions[0];
      glm::vec3 normal = glm::cross(edge1, edge2);
        if (backFaceCulling && normal.z <= 0) {
          return;  // Don't render back-facing triangles
      }


    for (int y = minCoords.y; y <= maxCoords.y; ++y) {
          for (int x = minCoords.x; x <= maxCoords.x; ++x) {

            glm::vec2 pixelCenter = glm::vec2(x + 0.5f, y + 0.5f);
            float denominator = ((positions[1].y - positions[2].y) * (positions[0].x - positions[2].x) + (positions[2].x - positions[1].x) * (positions[0].y - positions[2].y));

            
            float u = ((positions[1].y - positions[2].y) * (pixelCenter.x - positions[2].x) + (positions[2].x - positions[1].x) * (pixelCenter.y - positions[2].y)) / denominator;
            float v = ((positions[2].y - positions[0].y) * (pixelCenter.x - positions[2].x) + (positions[0].x - positions[2].x) * (pixelCenter.y - positions[2].y)) / denominator;
            float w = 1 - u - v;
            float w0 = vertices[0].gl_Position.w;
            float w1 = vertices[1].gl_Position.w;
            float w2 = vertices[2].gl_Position.w;

            glm::vec3 uncorrectedBarycentricCoords(u * w0, v * w1, w * w2);

            glm::vec3 correctedBarycentricCoords(
              uncorrectedBarycentricCoords.x / vertices[0].gl_Position.w,
              uncorrectedBarycentricCoords.y / vertices[1].gl_Position.w,
              uncorrectedBarycentricCoords.z / vertices[2].gl_Position.w);

            float barycentricSum = correctedBarycentricCoords.x + correctedBarycentricCoords.y + correctedBarycentricCoords.z;
            correctedBarycentricCoords /= barycentricSum;

            float z0 = vertices[0].gl_Position.z / vertices[0].gl_Position.w;
            float z1 = vertices[1].gl_Position.z / vertices[1].gl_Position.w;
            float z2 = vertices[2].gl_Position.z / vertices[2].gl_Position.w;

            float interpolatedDepth = z0 * correctedBarycentricCoords.x +
              z1 * correctedBarycentricCoords.y +
              z2 * correctedBarycentricCoords.z;



                if (correctedBarycentricCoords.x >= 0 && correctedBarycentricCoords.y >= 0 && correctedBarycentricCoords.z >= 0) {
                InFragment inFragment;
                inFragment.gl_FragCoord = glm::vec4(pixelCenter, interpolatedDepth, 1);

                inFragment.attributes[0].u1 = primitiveID;


                inFragment.attributes[0].v4 = glm::vec4(primitiveID,
                                                      *reinterpret_cast<float*>(&mem.uniforms[0].u1),
                                                      *reinterpret_cast<float*>(&mem.uniforms[0].i1),
                                                      mem.uniforms[0].v1);


                for (int i = 0; i < 3; ++i) {
                    inFragment.attributes[i].v4 = (vertices[0].attributes[i].v4 * correctedBarycentricCoords.x / vertices[0].gl_Position.w) +
                                                  (vertices[1].attributes[i].v4 * correctedBarycentricCoords.y / vertices[1].gl_Position.w) +
                                                  (vertices[2].attributes[i].v4 * correctedBarycentricCoords.z / vertices[2].gl_Position.w);
                    inFragment.attributes[i].v4 /= (correctedBarycentricCoords.x / vertices[0].gl_Position.w +
                                                    correctedBarycentricCoords.y / vertices[1].gl_Position.w +
                                                    correctedBarycentricCoords.z / vertices[2].gl_Position.w);
                }


                  OutFragment outFragment;
                  mem.programs[0].fragmentShader(outFragment, inFragment, tempShaderInterface);


                

                


                    // Check if the interpolated depth is less than the depth value currently stored in the depth buffer
                    int depthIdx = static_cast<int>(y * mem.framebuffer.width + x);

                    // Add a boolean variable to track whether the depth value should be updated
                    bool shouldUpdateDepth = interpolatedDepth <= mem.framebuffer.depth[depthIdx] && outFragment.gl_FragColor.a > 0.5f;

                    if (shouldUpdateDepth) {
                        // Update the depth buffer with the new depth value
                        mem.framebuffer.depth[depthIdx] = interpolatedDepth;
                    }


                    if (interpolatedDepth <= mem.framebuffer.depth[depthIdx] && outFragment.gl_FragColor.a > 0.5f) {
                    // Update the depth buffer with the new depth value
                    mem.framebuffer.depth[depthIdx] = interpolatedDepth;

                    // Write the output color to the framebuffer
                    int idx = depthIdx * mem.framebuffer.channels;
                   
                   
                  // Apply blending
                  float a = outFragment.gl_FragColor.a;
                  glm::vec4 frameColor(
                      static_cast<float>(mem.framebuffer.color[idx + 0]),
                      static_cast<float>(mem.framebuffer.color[idx + 1]),
                      static_cast<float>(mem.framebuffer.color[idx + 2]),
                      static_cast<float>(mem.framebuffer.color[idx + 3]));

                  // With these lines
                  glm::vec3 frameColorRGB = glm::vec3(frameColor.r, frameColor.g, frameColor.b) / 255.f;
                  glm::vec3 outputColor = glm::clamp(frameColorRGB * (1 - a) + glm::vec3(outFragment.gl_FragColor) * a, glm::vec3(0.f), glm::vec3(1.f)) * 255.f;

                  // Only write the output color to the framebuffer if the depth value should be updated
                  if (shouldUpdateDepth) {
                      mem.framebuffer.color[idx + 0] = static_cast<uint8_t>(outputColor.r);
                      mem.framebuffer.color[idx + 1] = static_cast<uint8_t>(std::round(outputColor.g));
                      mem.framebuffer.color[idx + 2] = static_cast<uint8_t>(std::round(outputColor.b));
                      mem.framebuffer.color[idx + 3] = static_cast<uint8_t>(frameColor.a);
                  }
              }
            }
        }
    }
}



                


void readAttributes(InVertex& inVertex, VertexArray& vao, bool indexed, GPUMemory* mem, uint32_t vertexID, uint32_t index) {
  for (int j = 0; j < maxAttributes; j++) {
    if (vao.vertexAttrib[j].type == AttributeType::EMPTY) {
      continue;
    }
    int32_t bufferID = vao.vertexAttrib[j].bufferID;
    uint64_t stride = vao.vertexAttrib[j].stride;
    uint64_t offset = vao.vertexAttrib[j].offset;

    if (indexed) {
      vertexID = index;
    }

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
    case AttributeType::UINT:
      inVertex.attributes[j].u1 = *(uint32_t*)(mem->buffers[bufferID].data + offset + vertexID * stride);
      break;
    }
  }
}

void runVertexAssembly(InVertex& inVertex, VertexArray& vao, bool indexed, GPUMemory& mem, uint32_t vertexID, uint32_t currentCommand) {
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
  }
  else {
    inVertex.gl_VertexID = vertexID;
  }
  inVertex.gl_DrawID = currentCommand;
  readAttributes(inVertex, vao, indexed, &mem, vertexID, inVertex.gl_VertexID);
}



void runPrimitiveAssembly(GPUMemory& mem, Program& prg, VertexArray& vao, OutVertex* outVertices, uint32_t vertexID, uint32_t primitiveID, uint32_t currentCommand, uint32_t nofVertices, DrawCommand const& drawCmd) {
    InVertex inVertices[3];
    ShaderInterface si;
    for (uint32_t i = 0; i < 3; ++i) {
        uint32_t index = vertexID + i;
        if (index >= nofVertices) {
            break;
        }
        runVertexAssembly(inVertices[i], vao, vao.indexBufferID != -1, mem, index, currentCommand);
        prg.vertexShader(outVertices[i], inVertices[i], si);
    }
    rasterizeTriangle(mem, outVertices, primitiveID, drawCmd);
}




void computeVertexID(InVertex& inVertex, uint32_t vertexID){
  inVertex.gl_VertexID = vertexID;
}






void draw(GPUMemory& mem, DrawCommand cmd, uint32_t currentCommand) {
    Program prg = mem.programs[cmd.programID];
    VertexArray vao = cmd.vao;
    uint32_t nofVertices = cmd.nofVertices;

    for (uint32_t i = 0; i < nofVertices; i += 3) {
        OutVertex outVertices[3];
        runPrimitiveAssembly(mem, prg, vao, outVertices, i, i / 3, currentCommand, nofVertices, cmd);
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
