# Sistema de Animação do Radion Blender

## Visão Geral

O sistema de animação permite criar, editar e playback de keyframe animations de meshes. A timeline e interpolação serão implementadas no próprio Blender, aplicando deformação via morph targets e skinning.

## Componentes

### 1. AnimationClip — Armazena Animação

```cpp
struct AnimationClip {
    ImGuiID clipId;                      // ID do clip
    std::string name;                    // "Walk", "Jump", etc
    float duration = 2.0f;               // segundos
    u32 fps = 24;
    bool loop = true;

    // Channels animados (vertex pos, rotation, etc)
    std::map<ImGuiID, std::vector<Keyframe>> keyframes;

    // Metadata
    bool hasBoneDeformation = false;     // skinning ativo
    bool hasMorphTargets = true;         // vertex morph
    std::vector<std::string> boneNames;  // skeleton info
};

struct Keyframe {
    float time;                          // segundos desde frame 0
    glm::vec3 value;                     // pos, rot, scale
    int easeType = iam_ease_linear;
    float bezierParams[4];               // se cubic_bezier
};
```

### 2. AnimationInstance — Playback

```cpp
struct AnimationInstance {
    ImGuiID instanceId;                  // ID da instância
    AnimationClip* clip;
    float playbackSpeed = 1.0f;
    bool playing = false;
    float currentTime = 0.0f;

    glm::vec3 getVertexOffset(u32 vertexIndex) const;
    glm::quat getBoneRotation(u32 boneIndex) const;
};
```

### 3. BlenderApplication Extensions

```cpp
class BlenderApplication {
private:
    AnimationClip mCurrentClip;
    AnimationInstance mCurrentInstance;
    std::map<std::string, AnimationClip> mClips;  // library

    // Deformation buffers
    std::vector<glm::vec3> mMorphOffsets;         // vertex offsets animados
    std::vector<glm::mat4> mBonePalette;          // transformações de bones

public:
    // Timeline authoring
    void insertKeyframe();                        // frame atual, selected verts
    void deleteKeyframe(u32 frame);
    void setKeyframeEasing(u32 frame, ImGuiID channel, int easeType);

    // Playback
    void playAnimation(const std::string& clipName);
    void pauseAnimation();
    void seekAnimation(float time);

    // Deformation queries
    glm::vec3 getAnimatedVertexPosition(u32 index) const;
    glm::mat4 getAnimatedBoneTransform(u32 index) const;

    // Persistence
    bool saveAnimation(const std::string& path);
    bool loadAnimation(const std::string& path);

private:
    void applyDeformation(f32 deltaTime);
    void recalculateNormals();
};
```

## Fluxo de Uso

### Fase: Criação de Animação

**1. Usuário seleciona verts e insere keyframe**

```cpp
// TimelinePanel::onImGui()
if (ImGui::Button("Insert Keyframe (I)")) {
    app().insertKeyframe();  // snapshot dos verts selecionados no frame atual
}
```

**2. BlenderApplication registra estado**

```cpp
void BlenderApplication::insertKeyframe() {
    u32 frame = mCurrentFrame;
    float time = frame / (f32)mFps;

    for (u32 vIdx : mSelection.selectedVertices()) {
        ImGuiID channel = makeChannelId("vertex", vIdx);
        
        // Registrar offset da posição original
        glm::vec3 offset = mMeshData->positions[vIdx] - mMeshDataOriginal->positions[vIdx];
        
        mCurrentClip.keyframes[channel].push_back({
            time,
            offset,
            iam_ease_linear
        });
    }

    recordUndo();
    markDirty();
}
```

**3. Usuário move frame, altera verts, insere novo keyframe**

(repete passo 1-2)

### Fase: Playback

**1. Usuário pressiona Play**

```cpp
// TimelinePanel::drawPlaybackControls()
if (ImGui::Button("Play")) {
    app().play();
}
```

**2. BlenderApplication cria instância**

```cpp
void BlenderApplication::play() {
    // Compilar o clip a partir de mCurrentClip
    iam_clip clip = iam_clip::begin(mCurrentClipId);

    for (auto& [channel, keyframes] : mCurrentClip.keyframes) {
        for (auto& kf : keyframes) {
            clip.key_vec3(channel, kf.time, kf.value, kf.easeType);
        }
    }
    clip.end();

    // Play
    mCurrentInstance.instanceId = ImGui::GetID("animation_playback");
    iam_play(mCurrentClipId, mCurrentInstance.instanceId);
    mPlaying = true;
}
```

**3. Frame update — query valores animados**

```cpp
void BlenderApplication::runFrame(f32 deltaTime) {
    if (mPlaying) {
        iam_update_begin_frame();
        iam_clip_update(deltaTime);

        // Query animated vertex offsets
        for (u32 v = 0; v < mMeshData->positions.size(); ++v) {
            ImGuiID channel = makeChannelId("vertex", v);
            float x, y, z;

            if (iam_instance.get_float(makeChannelId("vertex.x", v), &x) &&
                iam_instance.get_float(makeChannelId("vertex.y", v), &y) &&
                iam_instance.get_float(makeChannelId("vertex.z", v), &z)) {
                mMorphOffsets[v] = glm::vec3(x, y, z);
            }
        }

        applyDeformation(deltaTime);
        markDirty();
    }
}

void BlenderApplication::applyDeformation(f32 deltaTime) {
    // Copiar base mesh
    *mMeshData = mMeshDataOriginal;

    // Aplicar morph offsets
    for (u32 v = 0; v < mMeshData->positions.size(); ++v) {
        mMeshData->positions[v] += mMorphOffsets[v];
    }

    // Recalc normals se necessário
    recalculateNormals();

    // Upload ao GPU
    applyMeshEdit();
}
```

## Persistência (Save/Load)

### Formato JSON

```json
{
  "animation": {
    "name": "Extrude_Complex",
    "duration": 2.5,
    "fps": 24,
    "loop": true,
    "channels": {
      "vertex.42": {
        "type": "position",
        "keyframes": [
          {
            "time": 0.0,
            "value": [0.0, 0.0, 0.0],
            "easing": "linear"
          },
          {
            "time": 1.0,
            "value": [0.5, 1.0, 0.2],
            "easing": "out_cubic"
          },
          {
            "time": 2.5,
            "value": [0.0, 0.0, 0.0],
            "easing": "in_quad"
          }
        ]
      },
      "vertex.43": {
        "type": "position",
        "keyframes": [...]
      }
    }
  }
}
```

### API

```cpp
bool BlenderApplication::saveAnimation(const std::string& path) {
    nlohmann::json j;
    j["name"] = mCurrentClip.name;
    j["duration"] = mCurrentClip.duration;
    j["fps"] = mCurrentClip.fps;
    j["loop"] = mCurrentClip.loop;

    for (auto& [channel, keyframes] : mCurrentClip.keyframes) {
        auto& ch_json = j["channels"][std::to_string(channel)];
        for (auto& kf : keyframes) {
            nlohmann::json kf_json;
            kf_json["time"] = kf.time;
            kf_json["value"] = {kf.value.x, kf.value.y, kf.value.z};
            kf_json["easing"] = kf.easeType;
            ch_json["keyframes"].push_back(kf_json);
        }
    }

    std::ofstream out(path);
    out << j.dump(2);
    return true;
}
```

## Integração com Mini Renderer

O renderer precisa suportar:

1. **Vertex Buffers Dinâmicos** — atualizar positions/normals cada frame animado
2. **Morph Targets** — shader suporta offset positions
3. **Normal Recalc** — normais recalculadas após deformação

Ver `MINI_RENDERER.md` para detalhes.

## Timeline UI

```
┌─────────────────────────────────────────────────────┐
│ [Play] [Pause] Frame: 42 / 60  [Loop] Ease: ▼     │
└─────────────────────────────────────────────────────┘
│
│ Keyframe Track:
│ ─────────────────────────────────────────────────────
│ V 0  ●─────────────────────────────●──────────────●
│ V 1    ●───────────────────●
│ V 2        ●───────●─────────────────●
│ V 3              ●──────●
│       0         1         2        2.5
│ ─────────────────────────────────────────────────────
│       ↑ playhead @ 42 (1.75s)
```

## Roadmap

- [ ] Clip data structure & BlenderApplication integration
- [ ] TimelinePanel com inserção keyframes
- [ ] Playback básico (play/pause/seek)
- [ ] Display keyframes no timeline
- [ ] Deformation application (morph offsets)
- [ ] Normal recalculation
- [ ] Easing editor (bezier curve UI)
- [ ] Save/load animations (JSON)
- [ ] Copy/paste keyframes
- [ ] Bone deformation (skinning)
- [ ] Multiple animations library

---

**Notas:**
- Cada channel = vertex + component (x, y, z)
- ImGuiID para channel = hash estável (não deve mudar com undo/redo)
- Playback sempre a partir de cópia original do mesh (não acumular deformações)
- Normals recalc é custoso — só fazer se bounds mudaram muito
