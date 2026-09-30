# Plano: ferramentas de edição 3D (estilo MilkShape) e cliente de chat com LLM

Dois trabalhos ligados pela mesma API HTTP (`doc/API.md`): cada ferramenta nova do editor
passa a ser também um comando da API, e o cliente de chat usa esses comandos como
*tools* de um LLM. Por isso **nenhuma fase acaba sem o comando da API, o botão/atalho na
UI e testes**.

Legenda de tamanho: **S** = um comando/ferramenta pequena, **M** = várias peças ou UI
nova, **L** = subsistema novo.

Itens marcados **[confirmar]** dependem de informação que ainda não tenho (ver no fim).

---

## 1. Ponto de partida (lido do código, não suposto)

**Já existe** no editor: seleção de vértices e faces (caixa, grow/shrink, linked, invert),
gizmo de mover/rodar/escalar, extrude de faces, delete, weld, smooth, flip normals,
recalcular normais/tangentes, grupos (submeshes) com visibilidade, bisect, convex hull,
extrair submesh, UV planar/cilíndrica/esférica + unwrap (xatlas), simplify/optimize,
material por submesh (cor/rugosidade/metal), undo/redo, import (obj/fbx/gltf/rmesh),
export (rmesh/obj/glb), esqueleto e animações **só para reprodução** de ficheiros importados.

**Não existe** (ou é um esqueleto vazio):
- modo **Edge** (o menu diz "Edge selection is not implemented");
- **keyframes**: `insertKeyframe/deleteKeyframe/hasKeyframe` estão vazios;
- subdivisão, bevel/inset, loop cut, faca, fill/bridge, espelho/simetria, merge/separate;
- editor de UV com a textura à vista; pintura de texturas;
- criar esqueleto (juntas), pesos de skinning, animar;
- snap (grelha/vértice), esconder vértices/faces, 2D/ortho com seleção de caixa em todas as vistas **[confirmar]**.

## 2. O que fazia o MilkShape (de memória, **[confirmar]** com quem o usa)

Modelação low-poly rápida: vértice/aresta/face/grupo/junta como elementos selecionáveis,
4 vistas, extrude, subdivide, turn edge, flip, weld, snap to vertex/grid, mirror, merge,
primitivas, editor de coordenadas de textura, materiais, **juntas + pesos + keyframes**,
importação/exportação de muitos formatos, plugins. O que o tornava útil era a rapidez:
poucas ferramentas, todas ao alcance de um atalho.

---

## 3. Parte A — ferramentas de edição

### Fase A1 — Fundações (desbloqueiam tudo o resto) — M
| # | Item | Notas |
|---|---|---|
| A1.1 | **Modo aresta**: selecionar, mostrar, apagar | estrutura de arestas derivada dos triângulos; `BlenderSelection` ganha bits de aresta |
| A1.2 | **Esconder/mostrar** vértices e faces (H / Alt+H) | as ferramentas ignoram o que está escondido |
| A1.3 | **Snap** à grelha e a vértice, no gizmo e nos comandos | passo configurável |
| A1.4 | **Topologia** partilhada: mapa aresta→faces, vizinhança, bordos | base de A2–A3; testada à parte (como `MeshSelectionTests`) |

API: `select` ganha `mode:"edge"`; `hide`, `unhide`; `snap` como parâmetro de `transform_*`.

### Fase A2 — Ferramentas de modelação clássicas — L
| # | Item | Tamanho |
|---|---|---|
| A2.1 | **Subdivide** (dividir faces/arestas, e suavizado tipo Catmull-Clark/Loop) | M |
| A2.2 | **Turn/flip edge**, **split edge**, **collapse edge** | S cada |
| A2.3 | **Inset** e **bevel** | M |
| A2.4 | **Loop cut** + **faca** (plano) | M |
| A2.5 | **Fill hole / make face / bridge** entre dois bordos | M |
| A2.6 | **Espelho**: operação única e **simetria ao vivo** (editar um lado, o outro acompanha) | M — é o que mais poupa cliques nos veículos/aeronaves |
| A2.7 | **Merge / separate**: juntar partes, separar a seleção numa parte nova | S |
| A2.8 | **Booleanas** (união/diferença/interseção) reaproveitando `VolumeCSG` do motor | M **[confirmar]** qualidade/velocidade do CSG atual para malhas pequenas |
| A2.9 | Mais primitivas: disco, tubo/anel, escada, arco, prisma N-lados, **perfis extrudidos** (polígono 2D → sólido) | S cada |

Cada uma: comando na API, entrada nos menus Vertex/Edge/Face/Mesh, atalho, teste unitário
(contagens de vértices/triângulos, manifold/bordos com `analyzeMesh`, normais para fora).

### Fase A3 — Texturas e materiais — L
- **Editor de UV**: painel 2D com a textura, selecionar/mover/rodar/escalar ilhas, *pin*, projeções por seleção, caixa (box map). — L
- **Materiais com texturas**: atribuir ficheiro de albedo/normal/rugosidade a uma parte; o `.glb` passa a embutir as imagens. — M
- **Pintura de vértices** (cor por vértice) e cozinhar cor para textura. — M **[confirmar]** se interessa

### Fase A4 — Esqueleto e animação (o que falta para os helicópteros terem rotores a rodar) — L
- **Juntas**: criar, mover, ligar pai/filho, nomes. `Skeleton` já existe no motor.
- **Skinning**: atribuir vértices a juntas, pesos, pintura de pesos, normalização.
- **Keyframes**: implementar `insertKeyframe/delete/has` de verdade; curvas linear/constante; timeline editável; copiar/colar poses.
- **Export**: glTF com `skins` e `animations` (hoje o exportador é só geometria estática).
- API: `add_joint`, `set_joint`, `bind_part`, `set_weights`, `key_pose`, `play`, etc.

### Fase A5 — Conforto e formatos — M
- Importadores/exportadores em falta **[confirmar]** quais precisas (FBX export? MS3D import?).
- Vistas: layout 4-vistas com wireframe ortho, *zoom to selection*, atalhos configuráveis.
- Macros: gravar uma sequência de comandos da API e repetir (aproveita a API).

### Ordem recomendada
A1 → A2.6 (espelho) + A2.1 (subdivide) + A2.7 → restante A2 → A4 (keyframes primeiro) → A3.
Razão: espelho e subdivide dão o maior ganho por esforço; animação é o que desbloqueia o jogo;
UV/texturas só pesam quando houver arte texturizada.

---

## 4. Parte B — cliente de chat em Python + Qt

### Objetivo
Uma janela de chat: escreves "faz um tanque com torre a rodar", o LLM chama os comandos da
API, vê screenshots do resultado, corrige, e tu acompanhas e podes desfazer.

### Decisões de desenho
1. **Onde vive:** `blend/client/` (Python, separado do C++), `python -m radion_chat`.
2. **Binding Qt:** PySide6 (LGPL) — **[confirmar]** preferência por PyQt6 (GPL).
3. **Fornecedores de LLM atrás de uma interface única** (`LlmProvider`):
   - **OpenAI-compatível** (`/v1/chat/completions` com `tools`): cobre **Ollama**
     (`http://localhost:11434/v1`), **DeepSeek**, OpenAI, OpenRouter, LM Studio, vLLM.
     Configuras `base_url`, `model`, `api_key`.
   - **Anthropic** nativo (`/v1/messages` com `tools`).
   - Outros (Gemini, …) depois, sem mexer no resto.
4. **Tools:** o cliente lê `GET /api/commands` e transforma cada comando numa tool
   (nome, descrição, `inputSchema`). Nada de duplicar definições — comandos novos no
   editor aparecem no chat sem alterar o cliente.
5. **Ciclo do agente:** mensagem → LLM → `tool_calls` → `POST /api/commands/<nome>` →
   resultado volta ao LLM → repetir até o LLM responder em texto. Limite de passos por
   pedido, botão **Parar**, e botão **Desfazer tudo deste pedido** (n× `undo`).
6. **Imagens:** `screenshot` devolve PNG. Modelos com visão recebem-no; para modelos sem
   visão o cliente envia só o texto (`get_status`) e avisa na UI. A deteção é uma opção por
   modelo no perfil, não adivinhada.
7. **Modelos sem *tool calling* nativo** (alguns locais): modo alternativo em que o LLM
   responde com um bloco JSON `{"command":…, "arguments":…}` e o cliente executa.
   **[confirmar]** quais modelos usas — suporte a tools varia por modelo e por versão e
   não o verifiquei aqui.
8. **Chaves:** nunca em ficheiro em claro no repo. Ordem: variável de ambiente →
   `keyring` do sistema → campo na UI só em memória. Perfis guardam `base_url`/`model`,
   não a chave.
9. **Segurança:** comandos que escrevem ficheiros (`save_mesh`, `export_*`) e `new_document`
   pedem confirmação na UI por omissão (configurável); a API só aceita ligações locais e
   pode usar token (já suportado).
10. **Sessões:** histórico guardado (JSON) por ficheiro de trabalho; exportar conversa.
11. **Fornecedor MCP (fase B3):** além do fornecedor "API Radion" acima, um fornecedor MCP
    genérico (stdio/HTTP, SDK `mcp`) para usares o **servidor MCP que já tens** e outros.
    Até lá, o cliente não depende de MCP: fala direto com a API.

### Fases
| Fase | Entrega | Tamanho |
|---|---|---|
| B1 | Janela Qt: chat, perfil de LLM (base_url/model/chave), fornecedor OpenAI-compatível (Ollama/DeepSeek), tools a partir de `/api/commands`, ciclo do agente, screenshot no painel lateral, Parar | L |
| B2 | Anthropic nativo; modo JSON para modelos sem tools; Desfazer pedido; confirmações; histórico | M |
| B3 | Fornecedor MCP genérico; vários perfis; *prompts de sistema* por tarefa (veículo, edifício, personagem) com convenções da API; receitas guardadas ("macros" em linguagem natural) | M |

### Testes
- **Sem rede e sem GPU:** servidor LLM falso (HTTP local) que devolve *tool calls*
  determinísticas; o teste corre o ciclo completo contra um servidor da API falso
  (o `MainThreadQueue`/`ApiServer` C++ já têm testes; aqui basta um stub HTTP).
- Qt em `QT_QPA_PLATFORM=offscreen` para testar a janela/modelos.
- Teste de ponta a ponta manual documentado (editor + Ollama local).

---

## 5. Como vamos trabalhar
- Uma fase de cada vez, sem saltar testes nem a documentação (`doc/API.md` atualizado
  a cada comando novo).
- Cada fase termina com: comando(s) na API, UI, testes a passar (`ctest`), e um exemplo
  ou screenshot que o demonstre.
- Nada de PR sem tu pedires; commits pequenos no branch atual.

## 6. O que preciso de saber (não vou assumir)
1. **MilkShape:** quais são as 3–5 funções que mais usavas? (afina a ordem da Parte A.)
2. **Qt:** PySide6 ou PyQt6?
3. **Modelos:** que modelos queres usar primeiro no Ollama/DeepSeek? (decide o modo de
   tools e se precisamos já do modo JSON.)
4. **O teu servidor MCP:** linguagem e como arranca (stdio/HTTP)? (só para a fase B3)
5. **Formatos:** precisas de exportar FBX ou MS3D, ou basta `.glb`/`.obj`/`.rmesh`?
6. **Por onde começar:** recomendo **B1 + A1** em paralelo (o chat já fica útil com as
   ferramentas atuais; A1 dá a base às restantes).
