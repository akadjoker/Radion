# Arquitetura Completa — Radion Blender

## Estrutura Global

```
Radion Blender = Mini-editor de animações + meshes + viewport otimizado
├── Frontend (ImGui Panels)
│   ├── ViewportPanel      → Renderização 3D (1/3/4 vistas)
│   ├── TimelinePanel      → Keyframes + playback
│   ├── MeshEditPanel      → Vertex/face tools (extrude, scale, etc)
│   ├── HierarchyPanel     → Estrutura de meshes/bones
│   ├── PropertiesPanel    → Materials + export
│   └── ConsolePanel       → Logs
│
├── Backend (BlenderApplication)
│   ├── Selection (verts/faces)
│   ├── AnimationSystem    → Clip + Instance playback
│   ├── MeshData           → CPU copy sempre em RAM
│   ├── Undo/Redo          → State snapshots
│   └── Settings (persistence)
│
└── Rendering (MiniRenderer)
    ├── PBR Shader         → Simples, rápido, sem post-process
    ├── Directional Light  → 1 light + environment
    ├── Morph Targets      → Vertex deformation
    └── Multi-Viewport     → 4 vistas em paralelo @ 60fps
```

## Fluxos Principais

### Edição de Geometry

```
1. ViewportPanel (click) → selecionar vertex
2. BlenderSelection.selectVertex(idx)
3. MeshEditPanel (slider "Extrude Distance" ou tecla "E")
4. BlenderApplication.recordUndo()
5. Modificar mMeshData->positions[] em CPU
6. BlenderApplication.applyMeshEdit() → GPU upload
7. Viewport re-renders instantaneamente
```

### Criação de Animação

```
1. TimelinePanel (scrubber) → frame 0
2. Select verts no viewport
3. TimelinePanel ("Insert Keyframe" / tecla "I")
4. BlenderApplication.insertKeyframe()
   └─ Regista offsets dos vértices selecionados no clip atual
5. Mover para frame 24
6. Deformar verts (extrude, scale, etc)
7. Insert keyframe novamente
   └─ O clip passa a ter 2 keyframes, interpolados entre si
8. TimelinePanel (Play)
   └─ iam_clip_update() → query valores animados cada frame
   └─ applyDeformation() → posições verts interpoladas
   └─ GPU upload → viewport anima suavemente
```

### Viewport Rendering

```
1. MiniRenderer.renderViewport(index, meshData, camera)
   ├─ Setup camera (Perspective/Top/Front/Right)
   ├─ Frustum cull (verts fora da câmara)
   ├─ Batch por material
   ├─ Bind PBR shader
   ├─ Directional light + IBL (environment)
   └─ Render ao framebuffer @ 60fps
2. Multi-viewport (ViewportPanel layout mode)
   ├─ 1 view:   1920x1080
   ├─ 3 views:  tree layout (1 top, 2 bottom)
   └─ 4 views:  grid 2x2 (960x540 each)
```

## Componentes Chave

### BlenderApplication (Núcleo)

**Responsabilidades:**
- Gerenciar painéis
- Manter MeshData em CPU
- Controlar animação (clips e instâncias)
- Persistência (undo/redo, settings)
- Coordenar entre painéis (BlenderSelection, deformation, etc)

**Interface pública:**

```cpp
// Mesh
MeshData* currentMeshData();
bool loadMesh(const std::string& path);
bool applyMeshEdit();

// Animation
void insertKeyframe();
void playAnimation(const std::string& clipName);
void pauseAnimation();
float currentFrame() const;

// Selection
BlenderSelection& selection();

// Persistence
void recordUndo();
void undo();
void redo();
bool saveScene(const std::string& path);
```

### BlenderSelection (State)

Armazena quais verts/faces estão selecionados:

```cpp
enum SelectionMode { Vertex, Face, Edge };

void selectVertex(u32 index);
void toggleFace(u32 index);
bool isVertexSelected(u32 index) const;
const std::vector<u32>& selectedVertices() const;
```

### AnimationSystem

Fornece uma API de animação Blender-style:

```cpp
struct AnimationClip {
    ImGuiID clipId;
    std::map<ImGuiID, std::vector<Keyframe>> keyframes;
};

// Inside BlenderApplication:
AnimationClip mCurrentClip;
AnimationInstance mPlaybackInstance;

void insertKeyframe() { ... }
void playAnimation() { ... }
glm::vec3 getAnimatedVertexPosition(u32 idx) { ... }
```

### ViewportPanel (Rendering UI)

Multi-view com câmaras ortho/perspective:

```cpp
enum LayoutMode { Single, ThreeWay, FourWay };
enum ViewMode { Perspective, Top, Front, Back, Left, Right };

void drawSingleView();
void drawThreeWayLayout();
void drawFourWayLayout();
void drawViewportWindow(usize index, const char* name, ViewMode mode);
```

### TimelinePanel (Animation UI)

Timeline com scrubber, playback, keyframe insertion:

```cpp
void drawPlaybackControls();  // Play/Pause, frame slider, loop toggle
void drawTimelineRuler();     // Frame markers
void drawKeyframeTrack();     // Keyframe dots
```

### MiniRenderer (GPU)

PBR shader otimizado, render rápido, sem post-process:

```cpp
class MiniRenderer {
    void renderViewport(usize index, MeshData* mesh, Camera* cam);
    void setLightDirection(glm::vec3 dir);
    void setLightIntensity(float intensity);
};
```

Shader:
- **PBR.vert** — pos, normal, UV → WS
  - Suporta morph targets (vertex deformation)
  - Opcional: bone skinning (linear blend)
- **PBR.frag** — directional light + IBL
  - Albedo, normal map, roughness, metallic
  - No post-process (TAA, bloom, etc)

## Documentação

- **PLANO_BLENDER.md** — design inicial, painéis, features, regras
- **MINI_RENDERER.md** — spec do renderer (PBR, lighting, morph targets, performance)
- **ANIMATION_SYSTEM.md** — arquitetura da animação (clips, instances, deformation)
- **ARQUITECTURA.md** — este documento

## Roadmap

### Fase 1: Viewport + Seleção (MVP)

- [ ] MiniRenderer básico (PBR.vert/frag)
- [ ] ViewportPanel conectado ao renderer
- [ ] Picking (raycasting para seleção)
- [ ] Zoom/pan/orbit câmara
- [ ] Highlight de seleção

### Fase 2: Edição Básica

- [ ] Extrude (E key)
- [ ] Scale (S key)
- [ ] Rotate (R key)
- [ ] Smooth (Shift+S)
- [ ] Weld (W key)
- [ ] Delete (X key)
- [ ] Undo/Redo

### Fase 3: Timeline + Keyframes

- [ ] TimelinePanel com scrubber
- [ ] Insert/delete keyframes (I/Delete)
- [ ] Playback suave (interpolação)
- [ ] Play/pause/loop toggle

### Fase 4: Polish + Export

- [ ] Easing editor (bezier curve UI no timeline)
- [ ] Save/load animations (JSON)
- [ ] Export ao editor Radion
- [ ] Performance tuning (profiling)
- [ ] Material editor (slots, PBR params)

### Fase 5: Advanced (Nice-to-have)

- [ ] Bone deformation (skinning)
- [ ] Multiple animations library
- [ ] Copy/paste keyframes
- [ ] Markers & callbacks
- [ ] Motion paths (cubic splines)
- [ ] Stagger animations (per-vertex delay)

## Stack Tecnológico

- **Engine:** Radion (renderização, window, ImGui integration)
- **UI Framework:** ImGui (docking, panels, widgets)
- **Animation:** sistema de keyframes próprio
- **Math:** GLM (vec3, mat4, quat)
- **Serialization:** nlohmann::json (save/load)
- **Profiling:** Engine's built-in profiler (frame time, GPU metrics)

## Performance Targets

| Métrica | Target | Alvo |
|---------|--------|------|
| Viewport FPS | 60+ | Sem drops |
| 4-way render | ~4ms GPU | 960x540 cada |
| Mesh edit latency | <10ms | Click → highlight |
| Animation playback | Realtime | Sem stutter |
| Undo/redo latency | <1 frame | Instantâneo |

## Convenções de Código

Seguir CLAUDE.md do Radion:

- ✅ Sem comentários (zero menções a outros engines)
- ✅ Código + comentários em inglês
- ✅ Cabeçalhos: forward declaration, sem smart pointers
- ✅ Corpos de funções em .cpp (templates só em .h)
- ✅ Classes/managers singletons, sem funções soltas
- ✅ Sem lambdas em código quente
- ✅ Undo/redo via state snapshots (não patches)

## Próximos Passos

1. **Implementar MiniRenderer** (PBR.vert/frag, directional light)
2. **Conectar ViewportPanel** ao renderer
3. **Add picking** (raycasting para seleção de verts)
4. **Implement edit tools** (extrude, scale, rotate)
5. **Implementar playback de keyframes**
6. **Timeline UI** (scrubber, keyframe display, insertion)
7. **Save/load scenes** (persistence)
8. **Performance tuning** (profiling, optimization)

---

**Status:** Estrutura e documentação 100% pronta. Awaiting implementation. 🚀
