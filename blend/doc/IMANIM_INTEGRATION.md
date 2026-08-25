# ImAnim Integration para Radion Blender

## Overview

ImAnim oferece uma API completa de animação para ImGui. No contexto do Radion Blender, usamos seus widgets para criar uma timeline com keyframes de deformação de meshes.

## O que o ImAnim Oferece

### Clip System (Mais Relevante)
- **iam_clip** — fluent API para authoring de animações com keyframes
- **iam_instance** — playback control e query de valores animados
- Suporta keyframes com easing, repetição com variação, markers, callbacks

### Tweens (Para Preview)
- `iam_tween_float/vec2/vec4` — interpolação suave com easing
- Útil para preview suave entre keyframes no viewport

### Osciladores & Shake
- Motion organics em tempo real (útil para animações de efeitos)

### Paths & Morphing
- Animar ao longo de bezier curves (não relevante para mesh deformação, mas útil depois)

## Arquitetura de Integração

```
Radion Blender Animation Flow:
┌─────────────────────────────────────────────────┐
│ TimelinePanel (UI)                              │
│ - Display keyframes                             │
│ - Scrubber (frame slider)                       │
│ - Play/Pause                                    │
└──────────────────┬──────────────────────────────┘
                   │
                   ↓
        ┌──────────────────────┐
        │ BlenderApplication   │
        │ - mClips (iam_clip)  │ ← armazena animações criadas
        │ - mInstances (map)   │ ← playback em tempo real
        │ - currentMeshData    │
        └──────────────────────┘
                   │
    ┌──────────────┼──────────────┐
    ↓              ↓              ↓
┌─────────┐  ┌──────────┐  ┌──────────┐
│ Keyframe│  │  Easing  │  │ Callback │
│  Data   │  │  Type    │  │ (events) │
└─────────┘  └──────────┘  └──────────┘
```

## Conceitos do Radion Blender + ImAnim

### Channel = Vertex/Property Animada

Cada vertex selecionado (ou propriedade global como roughness) é um "channel":

```cpp
ImGuiID channel_id = ImHashStr("mesh.vertex[42].position");
// ou
ImGuiID channel_id = ImHashStr("mesh.material.roughness");
```

Cada frame podemos ter keyframes em canais diferentes:

```cpp
iam_clip clip = iam_clip::begin(clip_id);

// Frame 0: posição inicial
clip.key_vec3(pos_channel_0, 0.0f, glm::vec3(0, 0, 0), iam_ease_linear);
clip.key_vec3(rot_channel_0, 0.0f, glm::vec3(0, 0, 0), iam_ease_linear);

// Frame 24: extrude
clip.key_vec3(pos_channel_0, 1.0f, glm::vec3(0, 1, 0), iam_ease_out_cubic);

// Frame 48: return
clip.key_vec3(pos_channel_0, 2.0f, glm::vec3(0, 0, 0), iam_ease_in_cubic);

clip.end();
```

### Playback

Quando o utilizador pressiona "Play" no timeline:

```cpp
void BlenderApplication::play() {
    mPlaying = true;

    // Cada instância é um "take" (iteration) da animação
    iam_instance inst = iam_play(mCurrentClipId, mInstanceId);
    inst.set_time_scale(mPlaybackSpeed);
}
```

Durante cada frame:

```cpp
void BlenderApplication::runFrame(f32 dt) {
    if (mPlaying) {
        iam_clip_update(dt);  // Update all instances

        // Query valores animados por vertex
        for (u32 v = 0; v < mMeshData->positions.size(); ++v) {
            ImGuiID channel = ImHashStr(std::format("pos_{}", v).c_str());

            // Se este vertex tem keyframes, obter valor interpolado
            glm::vec3 animated_pos;
            float x, y, z;
            if (iam_instance.get_float(channel_x, &x) &&
                iam_instance.get_float(channel_y, &y) &&
                iam_instance.get_float(channel_z, &z)) {
                animated_pos = glm::vec3(x, y, z);
                mMeshData->positions[v] += animated_pos; // offset
            }
        }

        applyMeshEdit();  // Upload animada ao GPU
    }
}
```

## API Wrapper para BlenderApplication

Abstrair ImAnim por trás de uma interface limpa:

```cpp
// BlenderApplication.h
class BlenderApplication {
public:
    // Insert keyframe at current frame for selected vertices
    void insertKeyframe() {
        u32 frame = mCurrentFrame;
        for (u32 vIdx : mSelection.selectedVertices()) {
            ImGuiID channel = vertexPositionChannel(vIdx);
            addKeyframe(channel, frame, mMeshData->positions[vIdx]);
        }
        recordUndo();
        markDirty();
    }

    // Query animated value during playback
    glm::vec3 getAnimatedVertexPosition(u32 index) {
        // Lookup no iam_instance actual
        // Retornar pose interpolada (ou original se sem animação)
        ...
    }

private:
    ImGuiID mCurrentClipId;
    iam_instance mCurrentInstance;

    ImGuiID vertexPositionChannel(u32 index) {
        return ImHashStr(std::format("v.pos.{}", index).c_str());
    }

    void addKeyframe(ImGuiID channel, u32 frame, glm::vec3 value) {
        // Adicionar keyframe ao clip atual
        // (ou editar existente se frame já tem keyframe neste channel)
        ...
    }
};
```

## Integração com TimelinePanel

```cpp
// TimelinePanel.cpp
void TimelinePanel::drawKeyframeTrack() {
    ImDrawList* draw = ImGui::GetWindowDrawList();

    // Para cada channel com keyframes...
    for (auto& [channel, keyframes] : getAllKeyframes()) {
        for (auto& keyframe : keyframes) {
            float x = channelX + keyframe.time * pixelsPerFrame;
            float y = channelY + channelSpacing * channel.index;

            // Desenhar marcador de keyframe
            draw->AddCircleFilled({x, y}, 3.0f, IM_COL32(255, 180, 0, 255));

            // Hover = show info
            if (isMouseOver({x, y})) {
                ImGui::SetTooltip("Frame %u", keyframe.frame);
            }
        }
    }
}

void TimelinePanel::onImGui() {
    // Botão para inserir keyframe
    if (ImGui::Button("Insert Keyframe (I)")) {
        app().insertKeyframe();
    }

    // Timeline scrubber
    u32 frame = app().currentFrame();
    if (ImGui::SliderScalar("##frame", ImGuiDataType_U32, &frame, ...)) {
        app().setCurrentFrame(frame);

        // Scrub para esse frame na animação (seek)
        if (mCurrentInstance.valid()) {
            mCurrentInstance.seek(frame / 24.0f);  // 24 fps default
        }
    }
}
```

## Persistence (Save/Load Animações)

ImAnim oferece `iam_clip_save/load`, mas precisamos serializar também:
- Mapping de channels para vertex indices
- Metadata (durações, loop settings)

Formato JSON:

```json
{
  "animation": {
    "name": "Extrude_Test",
    "duration": 2.0,
    "fps": 24,
    "loop": true,
    "channels": {
      "v.pos.0": {
        "type": "vec3",
        "keyframes": [
          {"time": 0.0, "value": [0, 0, 0], "ease": "linear"},
          {"time": 1.0, "value": [0, 1, 0], "ease": "out_cubic"},
          {"time": 2.0, "value": [0, 0, 0], "ease": "in_cubic"}
        ]
      }
    }
  }
}
```

## Fase 1: MVP

- [ ] Wrapper de BlenderApplication para iam_clip (create, add keyframes)
- [ ] TimelinePanel com inserção de keyframes (botão "I")
- [ ] Playback básico (play/pause, seek)
- [ ] Display de keyframes no timeline
- [ ] Interpolação entre frames (preview suave)

## Fase 2: Polish

- [ ] Easing editor (bezier UI no timeline)
- [ ] Multi-channel keyframes (rotação, escala)
- [ ] Copy/paste keyframes
- [ ] Markers & callbacks (eventos na animação)
- [ ] Persist animações (save/load JSON)

## Fase 3: Advanced

- [ ] Layering (blend múltiplas animações)
- [ ] Stagger (delay entre keyframes de verts)
- [ ] Motion paths (animar ao longo de curve)
- [ ] Noise channels (organic movement)

---

## Notas

- ImAnim já está em `blend/vendor/ImAnim/` — pronto para usar
- Incluir `#include "im_anim.h"` no BlenderApplication.h
- Chamar `iam_update_begin_frame()` no inicio de BlenderApplication::runFrame()
- Chamar `iam_clip_update(dt)` após iam_update_begin_frame()
- GC periodicamente com `iam_clip_gc()` (opcional)

Ver `im_anim_usecase.cpp` e `im_anim_demo.cpp` para exemplos completos.
