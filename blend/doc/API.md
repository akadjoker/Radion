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
| `GET /api/commands` | `{"ok":true,"commands":[{name, description, readOnly, inputSchema}]}` |
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
`new_document`, `load_mesh`, `append_mesh`, `save_mesh` (formato `.rmesh`), `export_obj`, `undo`, `redo`

### Construir (cada um acrescenta uma parte)
| Comando | Para quê |
|---|---|
| `add_primitive` | `box`, `plane`, `sphere`, `cylinder`, `cone`, `capsule`, `torus`, já colocado e colorido |
| `add_lathe` | rotação de um perfil `[raio, y]` em torno de Y: cones, cúpulas, tanques |
| `add_loft` | secções transversais (elipse/superelipse) ao longo de um eixo: fuselagens, caudas, asas |
| `add_mesh` | vértices e triângulos em bruto; as normais são calculadas |

`add_primitive`, `add_lathe`, `add_loft` e `add_mesh` aceitam `position`, `rotation`, `scale`,
`name`, `color`, `roughness`, `metallic`.

### Editar partes
`transform_part`, `duplicate_part` (com `mirror: "x"|"y"|"z"`), `style_part`, `delete_part`,
`set_part_visible`, `extract_part`

### Editar geometria
`select` (por índices, parte ou caixa; `set/add/remove/clear/all/invert/grow/shrink/linked`),
`transform_selection`, `transform_mesh`, `extrude`, `delete_selection`, `weld_vertices`,
`smooth_vertices`, `recalculate_normals`, `flip_winding`, `center_mesh`, `bisect`,
`convex_hull`, `generate_uv`, `unwrap_uv`, `simplify`, `optimize`

### Animação
`set_animation` (clip, frame, playing) — para meshes com esqueleto.

## Exemplo

```bash
curl -s -X POST localhost:7420/api/commands/add_primitive \
  -d '{"type":"sphere","name":"corpo","color":"#c0392b","scale":[1,1,2.2]}'
```

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
blend/src/ProceduralShapes   lathe e loft (geometria pura, testada à parte)
blend/src/BlenderCapture     captura offscreen para o screenshot
```

Testes: `ctest -R blender_api` (registry, fila, servidor HTTP com cliente real, token,
Host/Origin) e `ctest -R procedural_shapes`.
