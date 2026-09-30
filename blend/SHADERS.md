# Shaders do MiniRenderer

## Overview

O MiniRenderer é independente do resto do engine: não passa por `GPU`/`AssetManager`,
usa OpenGL directo. Os shaders vivem como raw strings num namespace anónimo dentro de
`MiniRenderer.cpp` (não há `BlenderShaders.h` — foi removido, `.h` só declara).

Não são cópia de `docs/reference/`: não há PBR nenhum lá. São código novo e pequeno,
escrito para este preview, não um port de nada.

## Vertex Shader

### Entradas
```glsl
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;
layout(location = 3) in vec4 aTangent;  // xyz = tangent, w = handedness da bitangent
```

Sem `aBitangent` — `MeshData::tangents` (`runtime/render/include/Mesh.h`) só guarda
`vec4`, a bitangent é `cross(N, T) * tangent.w` no shader, como o resto do engine já faz.

Sem morph targets: `MeshData` não tem arrays de morph ainda, então não há atributo para
lhes ligar. Fica para quando essa deformação existir do lado dos dados.

### Saídas
```glsl
out VS_OUT {
    vec3 positionWS;
    vec3 normalWS;
    vec2 texCoord;
    mat3 TBN;
} vs_out;
```

## Fragment Shader

### Modos (`uShadingMode`)
- **0 — Solid**: `N.L` só, sem texturas, sem BRDF. Caminho barato para wireframe/solid
  e para desenhar muitos viewports/onion-skins ao mesmo tempo.
- **1 — Textured**: Cook-Torrance completo (Fresnel Schlick, GGX, Schlick-GGX), albedo/
  normal/roughness/metallic amostrados. Sem environment/IBL.

### Fallbacks (sem material carregado)
- Albedo/roughness/metallic → textura branca 1x1
- Normal → textura "flat" 1x1 (0.5, 0.5, 1.0)

Texturas reais de `MeshData::materialTextureFiles` ainda não são carregadas — está no
checklist do `doc/MINI_RENDERER.md`.

### Blending / onion-skin
`uAlpha` e `uTint` tingem e transparentam o resultado; quando `alpha < 1` o
`MiniRenderer::renderViewport` desliga o depth write e liga blending
(`SRC_ALPHA, ONE_MINUS_SRC_ALPHA`) antes do draw. O MiniRenderer não sabe o que é uma
pose fantasma — só desenha o que lhe passam com o alpha que lhe passam; ir buscar a pose
anterior/seguinte é trabalho do `TimelinePanel`/`ViewportPanel`.

### Wireframe
`MiniRenderMode::Wireframe` só liga `glPolygonMode(GL_LINE)` à volta do draw — usa o
mesmo shader em modo Solid.

### Debug views (`MiniDebugView`)
Substituem a shading normal por uma cor, antes de qualquer luz/textura:
- **Normals** (`uDebugView=1`) — `N*0.5+0.5`.
- **Tangents** (`uDebugView=2`) — primeira coluna do TBN, `T*0.5+0.5`.

`facetedShading` troca a normal suave (`vs_out.normalWS`) por uma `flat vec3
flatNormalWS` — a normal do vértice provocador do triângulo, sem dados extra nem
segundo shader, só um segundo varying sempre escrito. Afecta tanto o shading normal
(modo Solid) como a vista de Normals.

### Overlays extra (Blender-style)
Desenhados como segundo/terceiro draw call sobre o mesmo VAO, sem pipeline à parte:
- **`showWireframeOverlay`** — redesenha em `GL_LINE` por cima do preenchido, com
  `glPolygonOffset` para não fazer z-fight, cor quase preta. É o "ver as faces por cima
  do sólido" que o Blender faz com o overlay de wireframe.
- **`showVertexPoints`** — `glDrawArrays(GL_POINTS, 0, vertexCount)` direto do VBO
  (ignora o EBO), ponto a amarelo, `glPointSize(4)` fixo (sem `gl_PointSize` no shader,
  por isso `GL_PROGRAM_POINT_SIZE` fica desligado).

Seams (arestas de costura UV) ficaram de fora: `MeshData` não guarda essa informação —
teria de se detectar geometricamente (vértices na mesma posição com UVs diferentes),
que é trabalho à parte, não um toggle de shader.
