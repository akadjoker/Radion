# Radion Blender — Mini Mesh Editor

Blender-style mini-editor integrado ao Radion Engine, focado em edição de meshes e criação de animações keyframe.

## Estrutura

```
blend/
├── src/
│   ├── main.cpp                    # Entry point
│   ├── BlenderApplication.h/cpp    # Core application
│   ├── BlenderPanel.h              # Panel base class
│   ├── BlenderSelection.h/cpp      # Selection state (verts/faces)
│   ├── BlenderSettings.h/cpp       # Preferences persistence
│   ├── BlenderTheme.h              # ImGui theme (Blender orange/teal)
│   └── panels/
│       ├── ViewportPanel.h/cpp     # 1/3/4-way 3D viewport
│       ├── TimelinePanel.h/cpp     # Keyframe timeline
│       ├── MeshEditPanel.h/cpp     # Vertex/face editing tools
│       ├── HierarchyPanel.h/cpp    # Bone/mesh structure
│       ├── ConsolePanel.h/cpp      # Log messages
│       └── PropertiesPanel.h/cpp   # Materials & export
├── CMakeLists.txt
├── PLANO_BLENDER.md                # Architecture & design
├── MINI_RENDERER.md                # Render pipeline spec
└── README.md                        # This file
```

## Build

```bash
cd build/
cmake ..
make radion_blender
./bin/radion_blender
```

## Features

### Fase 1: Viewport + Seleção
- [x] Estrutura de painéis
- [x] Viewport multi-layout (1/3/4 vistas)
- [x] Selection state (verts/faces)
- [ ] Renderização 3D (Mini Renderer)
- [ ] Orbit câmara + gizmo seleção

### Fase 2: Edição Básica
- [ ] Extrude (E)
- [ ] Scale (S)
- [ ] Rotate (R)
- [ ] Smooth (Shift+S)
- [ ] Weld (W)
- [ ] Delete (X)

### Fase 3: Timeline + Keyframes
- [ ] Scrubbar timeline
- [ ] Insert/delete keyframes (I/Delete)
- [ ] Play/pause animação
- [ ] Interpolação entre frames

### Fase 4: Export + Polish
- [ ] Save pose como .rmesh
- [ ] Export animação como sequence
- [ ] Import no editor Radion

## Integração com Radion

- Usa `Engine` do Radion para renderização
- Carrega meshes via `AssetManager`
- Devolve `MeshData` editado
- Mesma infra de docking/settings que o editor

## Documentação

- **PLANO_BLENDER.md** — design, fluxo de painéis, regras
- **MINI_RENDERER.md** — spec do renderer otimizado (PBR sem post-process)

## Próximos Passos

1. Implementar Mini Renderer (PBR + directional light)
2. Conectar ViewportPanel ao renderer
3. Adicionar selection picking (raycasting)
4. Implementar edit tools (extrude, scale, etc)
5. Timeline & keyframe system
6. Export/import

---

**Status**: Estrutura base pronta. Aguardando implementação do Mini Renderer.
