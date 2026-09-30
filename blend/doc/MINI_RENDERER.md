# Mini Renderer para Blender

## Requisitos

- **Foco é performance, não qualidade** — é um preview, não uma referência de lighting
- **Independente do engine** — GL directo (`glCreateShader`/`glGenBuffers`/`glDrawElements`),
  sem passar por `GPU`/`AssetManager`/`Mesh`; o `PCH.h` mantém `<glad.h>` fora do resto do
  engine, mas o MiniRenderer é a excepção deliberada — a única razão de existir dele é ser
  um caminho de desenho mínimo e independente da pipeline pesada
- **Sem Post-Processing** — nada de TAA, bloom, tonemapping complexo
- **PBR Simples** — albedo, normal, roughness, metallic básicos, sem environment/IBL
- **Luz Mínima** — 1 directional light + cor ambiente (sem cubemap)
- **Modos de visualização** — Wireframe, Solid (N.L barato, sem texturas) e Textured (PBR completo)
- **Blending** — passo opaco (depth write) e passo transparente (alpha blend, sem depth write),
  usados também para onion-skin (poses fantasma da animação, tingidas e semi-transparentes)
- **Viewport Otimizado** — renderização rápida sem overdraw desnecessário
- **Multi-viewport** — render 4 viewports simultaneamente sem perda catastrófica de performance

## Pipeline de Render

```
Input: Mesh (CPU-side MeshData)
       ↓
Apply Deformation
  ├─ Morph Targets (blend verts animados)
  └─ Bone Skinning (se rigged)
       ↓
Cull & Batch (por material)
       ↓
Bind Shader (PBR.vert/frag)
       ↓
Render (single pass)
       ↓
Output: Framebuffer
```

## Deformação de Vertices

### Morph Targets (Animação Keyframe)

Cada keyframe da animação blender armazena uma pose de vertices:

```cpp
struct MorphTarget {
    std::vector<glm::vec3> positions;   // deformação offset
    std::vector<glm::vec3> normals;     // normais deformadas
    float weight = 1.0f;                // blend entre targets
};

// No update da animação (TimelinePanel.cpp):
void applyAnimationFrame(MeshData& mesh, const AnimationFrame& frame) {
    // Interpolar entre keyframes da timeline
    for (u32 v = 0; v < mesh.positions.size(); ++v) {
        glm::vec3 deformPos = sampleKeyframe(...);          // animada
        mesh.positions[v] += deformPos;                     // aplicar offset
    }
    
    // Recalc normals se deformação foi grande
    if (frame.recalcNormals) {
        recalculateNormals(mesh);
    }
}
```

### Bone Skinning (Rigged Models)

Se o mesh tem armature:

```cpp
struct Bone {
    glm::mat4 transform;
    u32 parentIndex;
    std::string name;
};

// No vert shader ou CPU:
vec4 deformed_pos = vec4(0);
for (int i = 0; i < 4; ++i) {
    mat4 bone_matrix = uBonePalette[boneIndices[i]];
    deformed_pos += bone_matrix * vec4(position, 1.0) * boneWeights[i];
}
```

Ou CPU-side para performance (morph + skinning):

```cpp
for (u32 v = 0; v < mesh.positions.size(); ++v) {
    glm::vec3 pos = mesh.positions[v];
    
    // 1. Apply morph
    pos += morphOffsets[v];
    
    // 2. Apply skinning
    glm::vec3 skinned = glm::vec3(0);
    for (int i = 0; i < 4; ++i) {
        skinned += mesh.bones[boneIndices[v*4+i]].matrix * pos * weights[v*4+i];
    }
    
    mesh.positions[v] = skinned;
}
```

### Shaders Necessários

**PBR.vert**
- Vertex input: position, normal, texcoord, (optional: bone indices + weights, morph targets)
- **Morph Targets** — suportar até 4 morph targets simultâneos (vertex deformation para animação)
  - Input: morphPosition0-3, morphNormal0-3, morphWeight0-3
  - Blend na CPU antes do render (ou via shader se performance permitir)
- **Bone Deformation** (linear blend skinning) — opcional para rigged meshes
  - Input: boneIndices (u8x4), boneWeights (f32x4)
  - Max 256 bones por frame
  - MatrixPalette buffer com transformações
- Saídas: pos WS, normal WS, texcoord

**PBR.frag**
- Input: normal WS, UV, position WS
- Sample: albedo, normal map, roughness, metallic
- Compute: directional light + IBL (environment probe)
- Output: tonemapped HDR → LDR (linear sRGB)

### Iluminação

**Directional Light**
- Intensity ajustável (slider)
- Direction ajustável (orbit gizmo no viewport)
- Toggle on/off (padrão: ligado)

**Ambiente**
- Cor + intensity ajustáveis, sem cubemap/IBL — one more texture fetch per pixel
  não se justifica num preview

**Sem sombras** — cascades CSM adiciona overhead não justificado para preview

## Otimizações

### CPU-side
- **Frustum culling** por viewport camera
- **Batch por material** (1 draw call per material)
- **No instancing** — simplicity first, um mesh por vez

### GPU-side
- **Early-Z** habilitado (discard em alpha-tested materials)
- **No MSAA** — edge antialiasing (edge-detect + blend post-cheap, ou aceitar jaggy)
- **Single-pass lighting** — sem deferred
- **Texture compression** — todos os assets já em BC1/BC4/BC5

### Framebuffer
- **1920x1080 shared** para os 4 viewports (scaling interno se multi-view)
  - 1 view: 1920x1080 inteiro
  - 3 views: 1920x1080 dividido
  - 4 views: 4x (960x540)
- **No MSAA** — swap speed over quality
- **sRGB backbuffer** — tonemapping trivial (gamma 2.2)

## Persistência de Preferências

```json
{
  "renderer": {
    "lightIntensity": 1.0,
    "lightDirection": [0.5, 1.0, 0.5],
    "ambientIntensity": 0.3,
    "enableDirectionalLight": true,
    "enableEnvironmentProbe": false,
    "environmentMapPath": ""
  }
}
```

Arquivo salvo em `.../Blender/renderer.settings.json`

## Integração com BlenderApplication

```cpp
class BlenderApplication {
  MiniRenderer mRenderer;

  void runFrame(f32 deltaTime) {
    mRenderer.setLightDirection(mLightDir);
    mRenderer.setLightIntensity(mLightIntensity);

    for (usize i = 0; i < numViewports; ++i) {
      mRenderer.renderViewport(i, mMeshData, camera[i]);
    }
  }
};
```

## Cheklist MVP

- [x] MiniRenderer class com init/shutdown (`blend/src/MiniRenderer.h/.cpp`)
- [x] Shaders GL directos, embutidos em `MiniRenderer.cpp` (sem ficheiro `.h` à parte,
      sem cópia de `docs/reference/` — não há PBR de referência lá; é código novo, pequeno,
      não um port)
- [x] Upload de mesh (VAO/VBO/EBO a partir de `MeshData`) e draw call real
- [x] Wireframe / Solid / Textured (`MiniRenderMode`)
- [x] Blending para passo transparente / onion-skin (`MiniDrawParams::alpha`, sem depth write)
- [ ] Render loop básico (1 viewport) — falta ligar ao `ViewportPanel`
- [ ] Multi-viewport rendering (3-way, 4-way)
- [ ] Frustum culling por objecto (fica ao lado de quem chama, MiniRenderer desenha 1 mesh
      por chamada e não conhece a cena)
- [ ] Directional light UI (intensity, direction)
- [ ] Normal map / roughness / metallic a partir de texturas reais (hoje usa fallbacks
      brancos/normal-plano — `MeshData::materialTextureFiles` ainda não é carregado)
- [ ] Settings persistence (JSON)
- [ ] Performance profiling (frame time display)

## Notas de Performance

**Alvo**: 60 FPS em 4 viewports @ 1920x1080 total (cascata 960x540 por view)

**Baseline**: por medir com o profiler quando o render loop estiver ligado ao
`ViewportPanel` — nenhum número abaixo foi medido, só metas.

Se não atingir, optimizações secundárias:
1. Reduzir resolução de textura (512px → 256px)
2. Disable normal maps (flat shading)
3. Frustum culling mais agressivo (por submesh)
4. Batching de draw calls (material sorting)
