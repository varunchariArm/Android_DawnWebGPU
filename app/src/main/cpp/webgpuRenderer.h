//
// Created by Varun Chari on 10/7/24.
//

#ifndef WEBGPUGAME_WEBGPURENDERER_H
#define WEBGPUGAME_WEBGPURENDERER_H

#include <android/log.h>
#include <memory>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <array>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_LEFT_HANDED
#include <glm/glm.hpp> // all types inspired from GLSL
#include <glm/ext.hpp>


#include <webgpu/webgpu.hpp>

//#include "save_image.h"

struct android_app;

/**
 * The same structure as in the shader, replicated in C++
 */
struct MyUniforms {
    // We add transform matrices
    glm::mat4x4 projectionMatrix;
    glm::mat4x4 viewMatrix;
    glm::mat4x4 modelMatrix;
    std::array<float, 4> color;
    float time;
    float _pad[3];
};

// Have the compiler check byte alignment
static_assert(sizeof(MyUniforms) % 16 == 0);

/**
 * A structure that describes the data layout in the vertex buffer
 * We do not instantiate it but use it in `sizeof` and `offsetof`
 */
struct VertexAttributes {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
};

class webgpuRenderer {
public:
    inline webgpuRenderer(android_app *pApp) :
            app_(pApp) {
        initWebgpuRenderer();
    }

    virtual ~webgpuRenderer();

    void render();

    void initWebgpuRenderer();

    wgpu::ShaderModule loadShaderModule(const std::filesystem::path& path, wgpu::Device device);
    bool loadGeometryFromObj(const std::filesystem::path& path, std::vector<VertexAttributes>& vertexData);

    android_app *app_;
    wgpu::Instance instance_;
    wgpu::Device device_;
    wgpu::Adapter adapter_;
    wgpu::ShaderModule shaderModule_;
    wgpu::RenderPipeline pipeline_;
    wgpu::Texture depthTexture_;
    wgpu::Texture texture_;
    wgpu::TextureView depthTextureView_;
    wgpu::Buffer vertexBuffer_;
    wgpu::Queue queue_;
    wgpu::SwapChain swapChain_;
    wgpu::Surface surface_;
    MyUniforms uniforms_;
    wgpu::Buffer uniformBuffer_;
    wgpu::Buffer indexBuffer_;
    std::vector<VertexAttributes> vertexData_;
    wgpu::BindGroup bindGroup_;
    std::vector<float> pointData_;
    std::vector<uint16_t> indexData_;
    int indexCount_;
    float angle1_;
    glm::mat4x4 R1_;
    glm::mat4x4 S_;
    glm::mat4x4 T1_;
    time_t last_time;
    time_t cur_time;

};

#endif //WEBGPUGAME_WEBGPURENDERER_H
