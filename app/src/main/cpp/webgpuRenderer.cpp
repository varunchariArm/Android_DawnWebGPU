#define WEBGPU_CPP_IMPLEMENTATION
#include "webgpuRenderer.h"
#include <android/log.h>
#include <game-activity/native_app_glue/android_native_app_glue.h>

#define TINYOBJLOADER_IMPLEMENTATION // add this to exactly 1 of your C++ files
#include "tiny_obj_loader.h"

using namespace wgpu;
namespace fs = std::filesystem;
using glm::mat4x4;
using glm::vec4;
using glm::vec3;
using glm::vec2;

constexpr float PI = 3.14159265358979323846f;

webgpuRenderer::~webgpuRenderer(){
    if(vertexBuffer_){
        vertexBuffer_.destroy();
        vertexBuffer_.release();
    }
    if(uniformBuffer_){
        uniformBuffer_.destroy();
        uniformBuffer_.release();
    }
    if(indexBuffer_){
        indexBuffer_.destroy();
        indexBuffer_.release();
    }
    if(texture_){
        texture_.destroy();
        texture_.release();
    }
    if(depthTextureView_){
        depthTextureView_.release();
    }
    if(depthTexture_){
        depthTexture_.destroy();
        depthTexture_.release();
    }
    if(pipeline_){
        pipeline_.release();
    }
    if(shaderModule_){
        shaderModule_.release();
    }
    if(queue_){
        queue_.release();
    }
    if(device_){
        device_.release();
    }
    if(adapter_){
        adapter_.release();
    }
    if(instance_){
        instance_.release();
    }
    if(surface_){
        surface_.release();
    }
    if(swapChain_){
        swapChain_.release();
    }
}

void webgpuRenderer::render() {
    time(&cur_time);
    auto diff = cur_time - last_time;
    uniforms_.time = static_cast<float>(diff);
    queue_.writeBuffer(uniformBuffer_, offsetof(MyUniforms, time), &uniforms_.time, sizeof(MyUniforms::time));
    // Update view matrix
    angle1_ = uniforms_.time;
    R1_ = glm::rotate(mat4x4(1.0), angle1_, vec3(0.0, 0.0, 1.0));
    uniforms_.modelMatrix = R1_ * T1_ * S_;
    queue_.writeBuffer(uniformBuffer_, offsetof(MyUniforms, modelMatrix), &uniforms_.modelMatrix, sizeof(MyUniforms::modelMatrix));
    SurfaceTexture targetSurfaceTexture;
    surface_.getCurrentTexture(&targetSurfaceTexture);
    Texture targetTexture = targetSurfaceTexture.texture;
    TextureView nextTexture = targetTexture.createView();
    if (!nextTexture) {
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "Cannot acquire next swap chain texture");
    }
    CommandEncoderDescriptor commandEncoderDesc;
    commandEncoderDesc.label = "Command Encoder";
    CommandEncoder encoder = device_.createCommandEncoder(commandEncoderDesc);

    RenderPassDescriptor renderPassDesc{};

    RenderPassColorAttachment renderPassColorAttachment{};
    renderPassColorAttachment.view = nextTexture;
    renderPassColorAttachment.resolveTarget = nullptr;
    renderPassColorAttachment.loadOp = LoadOp::Clear;
    renderPassColorAttachment.storeOp = StoreOp::Store;
    renderPassColorAttachment.clearValue = Color{ 1, 1, 1, 1.0 };
    renderPassColorAttachment.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
    renderPassDesc.colorAttachmentCount = 1;
    renderPassDesc.colorAttachments = &renderPassColorAttachment;

    RenderPassDepthStencilAttachment depthStencilAttachment;
    depthStencilAttachment.view = depthTextureView_;
    depthStencilAttachment.depthClearValue = 1.0f;
    depthStencilAttachment.depthLoadOp = LoadOp::Clear;
    depthStencilAttachment.depthStoreOp = StoreOp::Store;
    depthStencilAttachment.depthReadOnly = false;
    depthStencilAttachment.stencilClearValue = 0;
#ifdef WEBGPU_BACKEND_WGPU
    depthStencilAttachment.stencilLoadOp = LoadOp::Clear;
    depthStencilAttachment.stencilStoreOp = StoreOp::Store;
#else
    depthStencilAttachment.stencilLoadOp = LoadOp::Undefined;
    depthStencilAttachment.stencilStoreOp = StoreOp::Undefined;
#endif
    depthStencilAttachment.stencilReadOnly = true;

    renderPassDesc.depthStencilAttachment = &depthStencilAttachment;

    renderPassDesc.timestampWrites = nullptr;
    RenderPassEncoder renderPass = encoder.beginRenderPass(renderPassDesc);

    renderPass.setPipeline(pipeline_);

    renderPass.setVertexBuffer(0, vertexBuffer_, 0, vertexData_.size() * sizeof(VertexAttributes));
    // Set binding group
    renderPass.setBindGroup(0, bindGroup_, 0, nullptr);
    renderPass.draw(indexCount_, 1, 0, 0);

    renderPass.end();
    renderPass.release();

    nextTexture.release();

    CommandBufferDescriptor cmdBufferDescriptor{};
    cmdBufferDescriptor.label = "Command buffer";
    CommandBuffer command = encoder.finish(cmdBufferDescriptor);
    encoder.release();
    queue_.submit(command);
    command.release();
    surface_.present();
#ifdef WEBGPU_BACKEND_DAWN
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "device tick");
    device_.tick();
#endif
}

void webgpuRenderer::initWebgpuRenderer(){
    time(&last_time);
    instance_ = createInstance(InstanceDescriptor{});
    if (!instance_) {
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "WebGPU instance initialization failed");
    }
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Creating Surface");
    SurfaceDescriptorFromAndroidNativeWindow platformSurfaceDescriptor = {};
    platformSurfaceDescriptor.chain.next = nullptr;
    platformSurfaceDescriptor.chain.sType = SType::SurfaceDescriptorFromAndroidNativeWindow;
    platformSurfaceDescriptor.window = app_->window;
    SurfaceDescriptor surfaceDescriptor = {};
    surfaceDescriptor.label = "surfaceDescriptor";
    surfaceDescriptor.nextInChain = reinterpret_cast<const ChainedStruct*>(&platformSurfaceDescriptor);
    surface_ = instance_.createSurface(surfaceDescriptor);

    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Requesting adapter");
    RequestAdapterOptions adapterOpts{};
    adapterOpts.compatibleSurface = surface_;
    adapter_ = instance_.requestAdapter(adapterOpts);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Got adapter");
    AdapterInfo adapterInfo;
    adapter_.getInfo(&adapterInfo);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "vendor..");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", adapterInfo.vendor);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "architecture..");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", adapterInfo.architecture);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "device..");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", adapterInfo.device);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "description..");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", adapterInfo.description);
    std::string backend = std::to_string((int)adapterInfo.backendType);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "backendType..");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", backend.c_str());
    SupportedLimits supportedLimits;
    adapter_.getLimits(&supportedLimits);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Requesting device");
    RequiredLimits requiredLimits = Default;
    requiredLimits.limits.maxVertexAttributes = 3;
    requiredLimits.limits.maxVertexBuffers = 1;
    requiredLimits.limits.maxBufferSize = 10000 * sizeof(VertexAttributes);
    requiredLimits.limits.maxVertexBufferArrayStride = sizeof(VertexAttributes);
    requiredLimits.limits.minStorageBufferOffsetAlignment = supportedLimits.limits.minStorageBufferOffsetAlignment;
    requiredLimits.limits.minUniformBufferOffsetAlignment = supportedLimits.limits.minUniformBufferOffsetAlignment;
    requiredLimits.limits.maxInterStageShaderComponents = 6;
    requiredLimits.limits.maxBindGroups = 1;
    requiredLimits.limits.maxUniformBuffersPerShaderStage = 1;
    requiredLimits.limits.maxUniformBufferBindingSize = 16 * 4 * sizeof(float);
    requiredLimits.limits.maxTextureDimension1D = 480;
    requiredLimits.limits.maxTextureDimension2D = 640;
    requiredLimits.limits.maxTextureArrayLayers = 1;
    DeviceDescriptor deviceDesc;
    deviceDesc.label = "My Device";
    deviceDesc.requiredFeatureCount = 0;
    deviceDesc.requiredLimits = &requiredLimits;
    deviceDesc.defaultQueue.label = "The default queue";
    device_ = adapter_.requestDevice(deviceDesc);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Got device");
    static auto h = device_.setUncapturedErrorCallback([](ErrorType type, char const* message) {
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "Got device error");
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "error type:");
        std::string t = std::to_string((int)type);
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", t.c_str());
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "error message:");
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", message);
    });
    queue_ = device_.getQueue();
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Creating Swapchain");
#ifdef WEBGPU_BACKEND_WGPU
    TextureFormat swapChainFormat = surface_.getPreferredFormat(adapter_);
#else
    TextureFormat swapChainFormat = TextureFormat::Undefined;
    // Find suitable surface format.
    SurfaceCapabilities surfaceCapabilities;
    Status stat = surface_.getCapabilities(adapter_, &surfaceCapabilities);
    for(size_t ii = 0; ii < surfaceCapabilities.formatCount; ii++){
        if(surfaceCapabilities.formats[ii] == TextureFormat::RGBA8Unorm || surfaceCapabilities.formats[ii] ==TextureFormat::BGRA8Unorm){
            swapChainFormat = surfaceCapabilities.formats[ii];
            break;
        }
    }
    if (swapChainFormat == TextureFormat::Undefined) {
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "Could not find suitable surface format");
    }
    std::string form = std::to_string((int)swapChainFormat);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "swapChainFormat");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", form.c_str());

#endif
    TextureUsageFlags requiredTextureUsage = TextureUsage::RenderAttachment;
    if ((surfaceCapabilities.usages & requiredTextureUsage) != requiredTextureUsage) {
        __android_log_print(ANDROID_LOG_ERROR, "NATIVE", "%s", "Surface doesn't support the required texture usage.");
    }
    SurfaceConfiguration configuration = {};
    configuration.device = device_;
    configuration.width = 640;
    configuration.height = 480;
    configuration.format = swapChainFormat;
    configuration.usage = TextureUsage::RenderAttachment;
    configuration.presentMode = PresentMode::Mailbox ; // PresentMode::Fifo;
    configuration.alphaMode = surfaceCapabilities.alphaModes[0];
    surface_.configure(configuration);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "swap chain created");

    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Creating Shader module");
    shaderModule_ = loadShaderModule("/data/local/tmp/webgpu/shader_texture_file.wgsl", device_);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Created Shader module");
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Creating render pipeline");

    RenderPipelineDescriptor pipelineDesc;

    // Vertex fetch
    std::vector<VertexAttribute> vertexAttribs(3, Default);

    // Position attribute
    vertexAttribs[0].shaderLocation = 0;
    vertexAttribs[0].format = VertexFormat::Float32x3;
    vertexAttribs[0].offset = 0;

    // Normal attribute
    vertexAttribs[1].shaderLocation = 1;
    vertexAttribs[1].format = VertexFormat::Float32x3;
    vertexAttribs[1].offset = offsetof(VertexAttributes, normal);

    // Color attribute
    vertexAttribs[2].shaderLocation = 2;
    vertexAttribs[2].format = VertexFormat::Float32x3;
    vertexAttribs[2].offset = offsetof(VertexAttributes, color);

    VertexBufferLayout vertexBufferLayout;
    vertexBufferLayout.attributeCount = (uint32_t)vertexAttribs.size();
    vertexBufferLayout.attributes = vertexAttribs.data();
    vertexBufferLayout.arrayStride = sizeof(VertexAttributes);
    vertexBufferLayout.stepMode = VertexStepMode::Vertex;

    pipelineDesc.vertex.bufferCount = 1;
    pipelineDesc.vertex.buffers = &vertexBufferLayout;

    pipelineDesc.vertex.module = shaderModule_;
    pipelineDesc.vertex.entryPoint = "vs_main";
    pipelineDesc.vertex.constantCount = 0;
    pipelineDesc.vertex.constants = nullptr;

    pipelineDesc.primitive.topology = PrimitiveTopology::TriangleList;
    pipelineDesc.primitive.stripIndexFormat = IndexFormat::Undefined;
    pipelineDesc.primitive.frontFace = FrontFace::CCW;
    pipelineDesc.primitive.cullMode = CullMode::None;

    FragmentState fragmentState;
    pipelineDesc.fragment = &fragmentState;
    fragmentState.module = shaderModule_;
    fragmentState.entryPoint = "fs_main";
    fragmentState.constantCount = 0;
    fragmentState.constants = nullptr;

    BlendState blendState;
    blendState.color.srcFactor = BlendFactor::SrcAlpha;
    blendState.color.dstFactor = BlendFactor::OneMinusSrcAlpha;
    blendState.color.operation = BlendOperation::Add;
    blendState.alpha.srcFactor = BlendFactor::Zero;
    blendState.alpha.dstFactor = BlendFactor::One;
    blendState.alpha.operation = BlendOperation::Add;

    ColorTargetState colorTarget;
    colorTarget.format = swapChainFormat;
    colorTarget.blend = &blendState;
    colorTarget.writeMask = ColorWriteMask::All;

    fragmentState.targetCount = 1;
    fragmentState.targets = &colorTarget;

    DepthStencilState depthStencilState = Default;
    depthStencilState.depthCompare = CompareFunction::Less;
    depthStencilState.depthWriteEnabled = true;
    TextureFormat depthTextureFormat = TextureFormat::Depth24Plus;
    depthStencilState.format = depthTextureFormat;
    depthStencilState.stencilReadMask = 0;
    depthStencilState.stencilWriteMask = 0;

    pipelineDesc.depthStencil = &depthStencilState;

    pipelineDesc.multisample.count = 1;
    pipelineDesc.multisample.mask = ~0u;
    pipelineDesc.multisample.alphaToCoverageEnabled = false;

// Create binding layout (don't forget to = Default)
    BindGroupLayoutEntry bindingLayout = Default;
    bindingLayout.binding = 0;
    bindingLayout.visibility = ShaderStage::Vertex | ShaderStage::Fragment;
    bindingLayout.buffer.type = BufferBindingType::Uniform;
    bindingLayout.buffer.minBindingSize = sizeof(MyUniforms);

    // Create a bind group layout
    BindGroupLayoutDescriptor bindGroupLayoutDesc{};
    bindGroupLayoutDesc.entryCount = 1;
    bindGroupLayoutDesc.entries = &bindingLayout;
    BindGroupLayout bindGroupLayout = device_.createBindGroupLayout(bindGroupLayoutDesc);

    // Create the pipeline layout
    PipelineLayoutDescriptor layoutDesc{};
    layoutDesc.bindGroupLayoutCount = 1;
    layoutDesc.bindGroupLayouts = (WGPUBindGroupLayout*)&bindGroupLayout;
    PipelineLayout layout = device_.createPipelineLayout(layoutDesc);
    pipelineDesc.layout = layout;

    pipeline_ = device_.createRenderPipeline(pipelineDesc);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Created render pipeline");

    // Create the depth texture
    TextureDescriptor depthTextureDesc;
    depthTextureDesc.dimension = TextureDimension::_2D;
    depthTextureDesc.format = depthTextureFormat;
    depthTextureDesc.mipLevelCount = 1;
    depthTextureDesc.sampleCount = 1;
    depthTextureDesc.size = {640, 480, 1};
    depthTextureDesc.usage = TextureUsage::RenderAttachment;
    depthTextureDesc.viewFormatCount = 1;
    depthTextureDesc.viewFormats = (WGPUTextureFormat*)&depthTextureFormat;
    depthTexture_ = device_.createTexture(depthTextureDesc);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Created depth texture");

    // Create the view of the depth texture manipulated by the rasterizer
    TextureViewDescriptor depthTextureViewDesc;
    depthTextureViewDesc.aspect = TextureAspect::DepthOnly;
    depthTextureViewDesc.baseArrayLayer = 0;
    depthTextureViewDesc.arrayLayerCount = 1;
    depthTextureViewDesc.baseMipLevel = 0;
    depthTextureViewDesc.mipLevelCount = 1;
    depthTextureViewDesc.dimension = TextureViewDimension::_2D;
    depthTextureViewDesc.format = depthTextureFormat;
    depthTextureView_ = depthTexture_.createView(depthTextureViewDesc);
    __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Created depth texture view");

    // Load mesh data from OBJ file
    bool success = loadGeometryFromObj("/data/local/tmp/webgpu/cone_in_turdis.obj", vertexData_ /* dimensions */);
    if (!success) {
        __android_log_print(ANDROID_LOG_INFO, "NATIVE", "%s", "Could not load geometry!");
    }

    // Create vertex buffer
    BufferDescriptor bufferDesc;
    bufferDesc.size = vertexData_.size() * sizeof(VertexAttributes);
    bufferDesc.usage = BufferUsage::CopyDst | BufferUsage::Vertex;
    bufferDesc.mappedAtCreation = false;
    vertexBuffer_ = device_.createBuffer(bufferDesc);
    queue_.writeBuffer(vertexBuffer_, 0, vertexData_.data(), bufferDesc.size);

    indexCount_ = static_cast<int>(vertexData_.size());

    // Create uniform buffer
    bufferDesc.size = sizeof(MyUniforms);
    bufferDesc.usage = BufferUsage::CopyDst | BufferUsage::Uniform;
    bufferDesc.mappedAtCreation = false;
    uniformBuffer_ = device_.createBuffer(bufferDesc);

    // Build transform matrices
    // Translate the view
    vec3 focalPoint(0.0, 0.0, -1.0);
    // Rotate the object
    angle1_ = 2.0f; // arbitrary time
    // Rotate the view point
    float angle2 = 3.0f * PI / 4.0f;

    S_ = glm::scale(mat4x4(1.0), vec3(0.3f));
    T1_ = mat4x4(1.0);
    R1_ = glm::rotate(mat4x4(1.0), angle1_, vec3(0.0, 0.0, 1.0));
    uniforms_.modelMatrix = R1_ * T1_ * S_;

    mat4x4 R2 = glm::rotate(mat4x4(1.0), -angle2, vec3(1.0, 0.0, 0.0));
    mat4x4 T2 = glm::translate(mat4x4(1.0), -focalPoint);
    uniforms_.viewMatrix = T2 * R2;

    float ratio = 640.0f / 480.0f;
    float focalLength = 2.0;
    float near = 0.01f;
    float far = 100.0f;
    float divider = 1 / (focalLength * (far - near));
    uniforms_.projectionMatrix = transpose(mat4x4(
            1.0, 0.0, 0.0, 0.0,
            0.0, ratio, 0.0, 0.0,
            0.0, 0.0, far * divider, -far * near * divider,
            0.0, 0.0, 1.0 / focalLength, 0.0
    ));

    uniforms_.time = 1.0f;
    uniforms_.color = { 0.0f, 1.0f, 0.4f, 1.0f };
    queue_.writeBuffer(uniformBuffer_, 0, &uniforms_, sizeof(MyUniforms));

    // Create a binding
    BindGroupEntry binding{};
    binding.binding = 0;
    binding.buffer = uniformBuffer_;
    binding.offset = 0;
    binding.size = sizeof(MyUniforms);

    // A bind group contains one or multiple bindings
    BindGroupDescriptor bindGroupDesc;
    bindGroupDesc.layout = bindGroupLayout;
    bindGroupDesc.entryCount = bindGroupLayoutDesc.entryCount;
    bindGroupDesc.entries = &binding;
    bindGroup_ = device_.createBindGroup(bindGroupDesc);
}

ShaderModule webgpuRenderer::loadShaderModule(const fs::path& path, Device device) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return nullptr;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    std::string shaderSource(size, ' ');
    file.seekg(0);
    file.read(shaderSource.data(), size);

    ShaderModuleWGSLDescriptor shaderCodeDesc;
    shaderCodeDesc.chain.next = nullptr;
    shaderCodeDesc.chain.sType = SType::ShaderModuleWGSLDescriptor;
    shaderCodeDesc.code = shaderSource.c_str();
    ShaderModuleDescriptor shaderDesc;
    shaderDesc.nextInChain = &shaderCodeDesc.chain;
#ifdef WEBGPU_BACKEND_WGPU
    shaderDesc.hintCount = 0;
	shaderDesc.hints = nullptr;
#endif

    return device.createShaderModule(shaderDesc);
}

bool webgpuRenderer::loadGeometryFromObj(const fs::path& path, std::vector<VertexAttributes>& vertexData) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    // Call the core loading procedure of TinyOBJLoader
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.string().c_str());

    // Check errors
    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        return false;
    }

    // Filling in vertexData:
    vertexData.clear();
    for (const auto& shape : shapes) {
        size_t offset = vertexData.size();
        vertexData.resize(offset + shape.mesh.indices.size());

        for (size_t i = 0; i < shape.mesh.indices.size(); ++i) {
            const tinyobj::index_t& idx = shape.mesh.indices[i];

            vertexData[offset + i].position = {
                    attrib.vertices[3 * idx.vertex_index + 0],
                    -attrib.vertices[3 * idx.vertex_index + 2],
                    attrib.vertices[3 * idx.vertex_index + 1]
            };

            vertexData[offset + i].normal = {
                    attrib.normals[3 * idx.normal_index + 0],
                    -attrib.normals[3 * idx.normal_index + 2],
                    attrib.normals[3 * idx.normal_index + 1]
            };

            vertexData[offset + i].color = {
                    attrib.colors[3 * idx.vertex_index + 0],
                    attrib.colors[3 * idx.vertex_index + 1],
                    attrib.colors[3 * idx.vertex_index + 2]
            };

//            vertexData[offset + i].uv = {
//                    attrib.texcoords[2 * idx.texcoord_index + 0],
//                    1 - attrib.texcoords[2 * idx.texcoord_index + 1]
//            };
        }
    }

    return true;
}


