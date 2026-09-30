# API HTTP do Radion Blender

O editor pode expor uma API HTTP/JSON local para que scripts, ferramentas e modelos
de linguagem (LLM) o controlem: criar e editar uma mesh, pedir screenshots para
ver o resultado, guardar ficheiros. Um servidor MCP (ou qualquer outro cliente)
fala com esta API; o protocolo MCP em si não vive dentro do editor.

## Arrancar

```bash
radion_blender --api                       # porta 7420
radion_blender --api-port 8080
RADION_BLENDER_API_TOKEN=segredo radion_blender --api
```

| Opção | Efeito |
|---|---|
| `--api` | liga a API com o editor |
| `--api-port <n>` | porta (por omissão 7420); implica `--api` |
| `--api-host <addr>` | endereço onde escuta (por omissão `127.0.0.1`); implica `--api`. Fora de loopback **exige** token |
| `--api-token <s>` | exige `Authorization: Bearer <s>` (ou `RADION_BLENDER_API_TOKEN`, que não aparece na lista de processos) |
| `--no-api` | não liga, mesmo que esteja ativa nas Preferências |

Também em **Windows > Preferences > API**: porta, token, Start/Stop e "Start with the
editor". O token nunca é guardado em ficheiro. A barra de estado mostra `API :7420`
enquanto estiver a servir.

## Protocolo

| Pedido | Resposta |
|---|---|
| `GET /api/health` | `{"ok":true,"name":"radion_blender","apiVersion":1}` (sem token) |
| `GET /api/commands` | `{"ok":true,"commands":[{name, description, readOnly, undoable, inputSchema}]}` |
| `POST /api/commands/<nome>` | corpo = objeto JSON com os argumentos (vazio = `{}`) |

Sucesso: `{"ok":true,"result":{...}}`. Comandos que devolvem uma imagem (`screenshot`)
acrescentam `"image":{"mimeType":"image/png","data":"<base64>"}`.

Erro: `{"ok":false,"error":{"code":"...","message":"..."}}`

| HTTP | `code` | Significado |
|---|---|---|
| 400 | `invalid_params` | argumento em falta, tipo errado, fora de gama |
| 401 | `invalid_params` | token em falta ou errado |
| 403 | — | `Host`/`Origin` não local |
| 404 | `unknown_command` | comando inexistente |
| 422 | `failed` | pedido válido mas a operação não foi possível (ex.: nada selecionado) |
| 503 | `unavailable` | o editor está a fechar |
| 504 | `timeout` | o editor não respondeu a tempo (30 s) — p.ex. um diálogo modal aberto |

`GET /api/commands` traz o JSON Schema de cada comando, por isso um servidor MCP
pode gerar as suas *tools* a partir dele em vez de as duplicar: cada comando é uma
tool, `inputSchema` é o schema, `description` a descrição; o resultado de
`POST` (texto `result` + imagem `image`) mapeia para o conteúdo da resposta da tool.

### Segurança

- Só escuta em `127.0.0.1` por omissão.
- Pedidos com `Host` que não seja local, ou com `Origin` estrangeiro (uma página web
  a tentar falar com o editor), são recusados — com ou sem token.
- Os comandos lêem e escrevem ficheiros da máquina do editor (`load_mesh`,
  `save_mesh`, `export_obj`): trata o token como uma chave dessa máquina.

### Threads

Os pedidos chegam noutra thread mas **executam na thread do editor**, uma vez por
frame, pela ordem de chegada — veem e alteram exatamente o mesmo documento que os
painéis, e cada comando é um passo de undo.

## Convenções

- Unidades: as do editor. **+Y é para cima.** Um modelo "de frente" olha para **+Z**.
- `position` = `[x,y,z]`; `rotation` = graus, ângulos de Euler aplicados em X, depois Y,
  depois Z; `scale` = número (uniforme) ou `[x,y,z]` (não-uniforme: esfera → elipsoide;
  `-1` espelha).
- **Partes.** Cada peça adicionada é um submesh com o seu material; o nome do material é o
  nome da parte. Qualquer comando com `part` aceita o índice **ou** o nome.
- **Cores** em sRGB: `"#rrggbb"` ou `[r,g,b]` com 0..1. O editor converte para linear.
- Comandos de seleção/edição de vértices precisam de uma seleção (`select`); recusam
  (`failed`) em vez de editarem a mesh toda por engano.

## Comandos

### Ver
| Comando | O que faz |
|---|---|
| `get_status` | tudo: vértices, triângulos, bounds, partes, seleção, animação, undo/redo |
| `get_mesh_data` | posições e triângulos de uma parte (ou da mesh), para verificação |
| `screenshot` | PNG do modelo. `view` (perspective/front/back/left/right/top/bottom), `azimuth`/`elevation`, `frame`/`target`/`distance`, `shading` (textured/solid/wireframe), `wireframe_overlay`, `color_by_part`, `grid`, `width`/`height` |
| `get_selection` | modo, contagens, bounds e primeiros índices |

### Documento
`new_document`, `load_mesh`, `append_mesh`, `save_mesh` (formato `.rmesh`), `export_obj`,
`export_gltf` (`.glb`: uma primitiva e um material PBR por parte; embute as texturas PNG/JPEG
atribuídas com `set_texture` e escreve as cores de vértice como `COLOR_0`; só geometria
estática — sem esqueleto nem animação; o que não deu para embutir vem em `warnings`),
`undo`, `redo`

### Construir (cada um acrescenta uma parte)
| Comando | Para quê |
|---|---|
| `add_primitive` | `box`, `plane`, `sphere`, `cylinder`, `cone`, `capsule`, `torus`, `disc`, `tube`, `prism`, `stairs`, `arch`, já colocado e colorido. Todas ficam **centradas** na origem (a caixa envolvente); `origin: "base"` pousa-as em y=0 |
| `add_lathe` | rotação de um perfil `[raio, y]` em torno de Y: cones, cúpulas, tanques |
| `add_loft` | secções transversais (elipse/superelipse) ao longo de um eixo: fuselagens, caudas, asas |
| `add_mesh` | vértices e triângulos em bruto; as normais são calculadas |

`add_primitive`, `add_lathe`, `add_loft` e `add_mesh` aceitam `position`, `rotation`, `scale`,
`name`, `color`, `roughness`, `metallic`.

### Editar partes
`transform_part`, `duplicate_part` (com `mirror: "x"|"y"|"z"`), `style_part`, `delete_part`,
`set_part_visible`, `extract_part`

### Seleção e visibilidade
`select` (modos `vertex`/`edge`/`face`; por índices, parte ou caixa;
`set/add/remove/clear/all/invert/grow/shrink/linked`), `hide` (a seleção, ou tudo o que não
está selecionado) e `unhide`. Os triângulos escondidos não se selecionam nem são tocados
pelas edições; uma edição que muda o número de triângulos mostra tudo outra vez.

### Editar geometria
`transform_selection`, `transform_mesh`, `extrude`, `delete_selection`, `weld_vertices`,
`smooth_vertices`, `recalculate_normals`, `flip_winding`, `center_mesh`, `bisect`,
`convex_hull`, `simplify`, `optimize`, `snap_to_grid` (passo), `snap_to_vertex` (tolerância)

### Topologia (estilo MilkShape)
| Comando | O que faz |
|---|---|
| `subdivide` | parte as faces (selecionadas, ou todas) `levels` vezes; `smooth` = Loop (arredonda) |
| `turn_edge`, `split_edge`, `collapse_edge` | viram a diagonal, acrescentam um vértice (`t`), fundem as pontas de cada aresta selecionada |
| `knife` | corta por um plano (`axis`+`offset` ou `normal`), guardando a geometria; a linha nova fica selecionada |
| `loop_cut` | seleciona UMA aresta atravessada ao anel e acrescenta `cuts` laços |
| `inset`, `bevel` | `inset` encolhe a região selecionada (`thickness`, `depth`); `bevel` chanfra arestas selecionadas (`width`) |
| `fill_holes`, `bridge` | tampam bordos abertos; `bridge` liga dois bordos com uma tira |
| `mirror` | acrescenta a imagem espelhada das faces (eixo + `offset`, soldando `weld`) |
| `set_symmetry` | simetria ao vivo: editar um lado move o outro (`axis: "none"` desliga) |
| `merge_parts`, `separate_selection` | juntam partes numa só; transformam as faces selecionadas numa parte nova |
| `boolean` | `union`/`difference`/`intersection` entre duas partes (`a`, `b`): remalha um volume numa grelha de `resolution` células (8–160) — estanque e suave mas não é um corte exato; só sólidos fechados; evitar faces exatamente coplanares |

### Texturas e cores
| Comando | O que faz |
|---|---|
| `set_texture`, `clear_texture` | põem/tiram uma imagem (`albedo`, `normal`, `surface` = metal-rugosidade glTF, `emissive`) no material de uma parte; `get_status` mostra `textures` por parte |
| `paint_vertices` | pinta cor por vértice (`color` sRGB, `opacity`): `target` = `selection`, `part`, `all` ou `sphere` (pincel redondo com `center`, `radius`, `hardness`). A cor multiplica o material — pintar sobre material branco; vê-se no viewport, nos screenshots (`vertex_colors`) e no `.glb` (`COLOR_0`, linear como o glTF manda) |
| `clear_vertex_colors` | apaga a cor pintada (`selection`, `part` ou tudo) |

### UV
A origem das UV é o **canto superior esquerdo** da imagem (como nos ficheiros de textura e no
glTF): `v` cresce para baixo.

| Comando | O que faz |
|---|---|
| `generate_uv`, `unwrap_uv` | projeção planar/cilíndrica/esférica; ilhas sem sobreposição (xatlas) — sobre a mesh toda |
| `box_map_uv` | projeção em caixa (`tile` repetições por unidade, `offset`) de `selection`/`island`/`part`/`all`; parte vértices partilhados por faces que olham para lados diferentes |
| `transform_uv` | move/roda/escala/espelha UV (`translate`, `rotate`, `scale`, `flip`, `pivot`) de `selection`/`island`/`part`/`all` |
| `fit_uv` | ajusta ao quadrado 0..1; `per_part: true` dá a cada parte o seu quadrado (texturas por parte, depois de um `unwrap_uv` único) |
| `pin_uv` | fixa vértices para `transform_uv` e `fit_uv` os deixarem em paz |
| `get_uv_data` | limites, nº de ilhas, se cabe em 0..1, e as UV por vértice |
| `uv_layout` | PNG da malha UV de uma parte sobre a sua textura — para *ver* um unwrap |

O editor tem um painel **UV Editor** (ao lado do viewport) e um menu **Paint**.

### Animação
`set_animation` (clip, frame, playing) — para meshes com esqueleto.

### Undo
Cada comando que altera o documento é **um** passo de undo; `get_status` traz `undoSteps`.
No listado de comandos, `undoable` diz se uma chamada bem-sucedida acrescenta um passo
(`false` para os só de leitura e para `select`, `hide`, `unhide`, `set_part_visible`,
`set_animation`, `set_symmetry`, `pin_uv`, os que escrevem ficheiros, e os que esvaziam a pilha:
`new_document`, `load_mesh`, `undo`, `redo`). `tools/api_selftest.py` verifica a flag contra o
editor a cada comando que corre.

## Exemplo

```bash
curl -s -X POST localhost:7420/api/commands/add_primitive \
  -d '{"type":"sphere","name":"corpo","color":"#c0392b","scale":[1,1,2.2]}'
```

`blend/examples/game_shapes.py` constrói os modelos voadores de um jogo (helicóptero do jogador,
helicóptero inimigo, caça e foguetão) e `blend/examples/game_ground.py` os de terra (camião de
mísseis, radar, barraca de lona e barracão de chapa); cada um é guardado em `.glb` + `.rmesh` +
screenshot. Os modelos voadores ficam centrados na origem, os de terra assentam em y = 0.

`blend/examples/blender_api_client.py` é um cliente mínimo (só biblioteca padrão) e
`blend/examples/helicopter.py` constrói um helicóptero completo e guarda screenshots:

```bash
radion_blender --api &
python3 blend/examples/helicopter.py out/
```

## Por dentro

```
blend/src/api/
  CommandRegistry   comandos: nome, schema, handler; CommandArgs valida os argumentos
  MainThreadQueue   passa o trabalho da thread HTTP para a thread do editor
  ApiServer         servidor HTTP (cpp-httplib), autenticação, Host/Origin
  BlenderCommands   os comandos do editor, sobre as operações do BlenderApplication
  BlenderApiHost    liga tudo e é o dono; o BlenderApplication chama pump() por frame
blend/src/ProceduralShapes   lathe, loft, extrusão, disco, tubo, prisma, escada, arco (geometria pura)
blend/src/mesh/              MeshTopology, MeshEdit, MeshBoolean, MeshPaint, MeshUv (puro, sem GL)
blend/src/GltfExporter       escrita do .glb (texturas embutidas, COLOR_0)
blend/src/BlenderCapture     captura offscreen para o screenshot
blend/src/panels/UvEditorPanel   o painel de UV
blend/client/                cliente de chat em Python + Qt (ver blend/client/README.md)
```

Testes (`ctest`): `blender_api` (registry, fila, servidor HTTP com cliente real, token,
Host/Origin), `procedural_shapes`, `mesh_topology`, `mesh_edit`, `mesh_boolean`, `blender_uv_edit`,
`mesh_paint`, `gltf_export`, `blender_selection`. De ponta a ponta, com o editor a correr:
`python3 blend/tools/api_selftest.py --launch bin/radion_blender` (precisa de `xvfb-run` sem
ecrã) — corre todos os grupos de comandos e confere a flag `undoable`.
