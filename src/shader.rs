use vecmath::Matrix4;
use wgpu::winit::Window;

pub const VS_BYTES : &[u8] = include_bytes!("block.vert.spv");
pub const FS_BYTES : &[u8] = include_bytes!("block.frag.spv");

pub struct SizedBuffer{
    pub buffer: wgpu::Buffer,
    pub size: u32,
}

#[derive(Clone, Copy)]
pub struct Vertex {
    pub xyz: [f32; 3],
    pub uv: [f32; 2],
    pub rgb: [f32; 3],
}

pub struct Renderer {
    pipeline: wgpu::RenderPipeline,
    swap_chain: wgpu::SwapChain,

    bind_group: wgpu::BindGroup,

    _texture_view: wgpu::TextureView,
    depth_texture_view: wgpu::TextureView,
    mx_buffer: wgpu::Buffer,
    _sampler: wgpu::Sampler,

    matrix: [Matrix4<f32>; 2],
}

impl Renderer {
    const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::D32Float;

    pub fn new(device: &wgpu::Device, window: &Window, instance: &wgpu::Instance, texture: &wgpu::Texture) -> Self {
        let vs_module = device.create_shader_module(VS_BYTES);
        let fs_module = device.create_shader_module(FS_BYTES);

        // Create pipeline layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            bindings: &[
                wgpu::BindGroupLayoutBinding {
                    binding: 0,
                    visibility: wgpu::ShaderStageFlags::VERTEX,
                    ty: wgpu::BindingType::UniformBuffer,
                },
                wgpu::BindGroupLayoutBinding {
                    binding: 1,
                    visibility: wgpu::ShaderStageFlags::FRAGMENT,
                    ty: wgpu::BindingType::SampledTexture,
                },
                wgpu::BindGroupLayoutBinding {
                    binding: 2,
                    visibility: wgpu::ShaderStageFlags::FRAGMENT,
                    ty: wgpu::BindingType::Sampler,
                },
            ],
        });

        let size = window
            .get_inner_size()
            .unwrap()
            .to_physical(window.get_hidpi_factor());

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsageFlags::OUTPUT_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8Unorm,
            width: size.width.round() as u32,
            height: size.height.round() as u32,
        };

        let texture_view = texture.create_default_view();

        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            size: wgpu::Extent3d {
                width: sc_desc.width,
                height: sc_desc.height,
                depth: 1,
            },
            array_size: 1,
            dimension: wgpu::TextureDimension::D2,
            format: Self::DEPTH_FORMAT,
            usage: wgpu::TextureUsageFlags::OUTPUT_ATTACHMENT,
        });

        let depth_texture_view = depth_texture.create_default_view();

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            r_address_mode: wgpu::AddressMode::Repeat,
            s_address_mode: wgpu::AddressMode::Repeat,
            t_address_mode: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::FilterMode::Nearest,
            lod_min_clamp: -100.0,
            lod_max_clamp: 100.0,
            max_anisotropy: 0,
            compare_function: wgpu::CompareFunction::Always,
            border_color: wgpu::BorderColor::OpaqueWhite,
        });

        let mx_buffer = device.create_buffer(
            &wgpu::BufferDescriptor{
                size: 32, // 16 for view, 16 for projection.
                usage: wgpu::BufferUsageFlags::UNIFORM | wgpu::BufferUsageFlags::TRANSFER_DST
            }
        );

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[
                wgpu::Binding {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer {
                        buffer: &mx_buffer,
                        range: 0..128, // 64 * 2
                    },
                },
                wgpu::Binding {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&texture_view),
                },
                wgpu::Binding {
                    binding: 2,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            layout: &pipeline_layout,
            vertex_stage: wgpu::PipelineStageDescriptor {
                module: &vs_module,
                entry_point: "main",
            },
            fragment_stage: wgpu::PipelineStageDescriptor {
                module: &fs_module,
                entry_point: "main",
            },
            rasterization_state: wgpu::RasterizationStateDescriptor {
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: wgpu::CullMode::Front,
                depth_bias: 0,
                depth_bias_slope_scale: 0.0,
                depth_bias_clamp: 0.0,
            },
            primitive_topology: wgpu::PrimitiveTopology::TriangleList,
            color_states: &[wgpu::ColorStateDescriptor {
                format: wgpu::TextureFormat::Bgra8Unorm,
                color: wgpu::BlendDescriptor::REPLACE,
                alpha: wgpu::BlendDescriptor::REPLACE,
                write_mask: wgpu::ColorWriteFlags::ALL,
            }],
            depth_stencil_state: Some(wgpu::DepthStencilStateDescriptor {
                    format: Self::DEPTH_FORMAT,
                    depth_write_enabled: true,
                    depth_compare: wgpu::CompareFunction::Less,
                    stencil_front: wgpu::StencilStateFaceDescriptor::IGNORE,
                    stencil_back: wgpu::StencilStateFaceDescriptor::IGNORE,
                    stencil_read_mask: 0,
                    stencil_write_mask: 0,
            }),
            index_format: wgpu::IndexFormat::Uint16,
            vertex_buffers: &[wgpu::VertexBufferDescriptor {
                stride: std::mem::size_of::<Vertex>() as u32,
                step_mode: wgpu::InputStepMode::Vertex,
                attributes: &[
                    // XYZ
                    wgpu::VertexAttributeDescriptor {
                        attribute_index: 0,
                        format: wgpu::VertexFormat::Float3,
                        offset: 0,
                    },
                    // UV
                    wgpu::VertexAttributeDescriptor {
                        attribute_index: 1,
                        format: wgpu::VertexFormat::Float2,
                        offset: 4 * 3,
                    },
                    // RGB
                    wgpu::VertexAttributeDescriptor {
                        attribute_index: 2,
                        format: wgpu::VertexFormat::Float3,
                        offset: 4 * 5,
                    },
                ],
            }],
            sample_count: 1,
        });

        let surface = instance.create_surface(&window);
        
        let swap_chain = device.create_swap_chain(
            &surface,
            &sc_desc,
        );

        Renderer {
            pipeline: render_pipeline,
            swap_chain,

            bind_group,

            mx_buffer,
            _sampler: sampler,
            _texture_view: texture_view,
            depth_texture_view,

            matrix: Default::default(),
        }
    }

    pub fn set_projection(&mut self, mut proj_mat: Matrix4<f32>) {
        for x in 0..4 {
            proj_mat[x][2] = (proj_mat[x][3]+proj_mat[x][2])/2.;
        }
        self.matrix[0] = proj_mat;
    }

    pub fn set_view(&mut self, mut view_mat: Matrix4<f32>) {
        for x in 0..4 {
            view_mat[x][1] *= -1.;
        }
        self.matrix[1] = view_mat;
    }

    pub fn render<F>(&mut self, device: &mut wgpu::Device, f: F) where F: FnOnce(&mut FnMut(&SizedBuffer)){
        let frame = self.swap_chain.get_next_texture();
        let mx_buffer = device
            .create_buffer_mapped(
                32,
                wgpu::BufferUsageFlags::UNIFORM | wgpu::BufferUsageFlags::TRANSFER_DST,
            )
            //TODO: Replace this unsafe with a better cast
            .fill_from_slice(&unsafe{std::mem::transmute::<_, [f32; 32]>(self.matrix)});
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { todo: 0 });
        {
            encoder.copy_buffer_to_buffer(&mx_buffer, 0, &self.mx_buffer, 0, 128);

            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &frame.view,
                    load_op: wgpu::LoadOp::Clear,
                    store_op: wgpu::StoreOp::Store,
                    clear_color: wgpu::Color{r: 0.81, g: 0.8, b: 1.0, a: 1.0},
                }],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachmentDescriptor {
                    attachment: &self.depth_texture_view,
                    depth_load_op: wgpu::LoadOp::Clear,
                    depth_store_op: wgpu::StoreOp::Store,
                    stencil_load_op: wgpu::LoadOp::Clear,
                    stencil_store_op: wgpu::StoreOp::Store,
                    clear_depth: 1.0,
                    clear_stencil: 0,
                }),
            });
            rpass.set_pipeline(&self.pipeline);
            rpass.set_bind_group(0, &self.bind_group, &[]);
            f(&mut |buff|{
                rpass.set_vertex_buffers(&[(&buff.buffer, 0)]);
                rpass.draw(0..buff.size, 0..1);
            })
        }

        device.get_queue().submit(&[encoder.finish()]);
    }
}
