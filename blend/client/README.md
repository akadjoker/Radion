# Radion Chat

Cliente de chat (Python + Qt) que deixa um modelo de linguagem (LLM) construir malhas 3D no
editor Radion Blender. Escreves "faz um tanque com torre a rodar"; o modelo chama os comandos
da API HTTP do editor (`blend/doc/API.md`), vê *screenshots* do resultado, corrige, e tu
acompanhas tudo e podes desfazer.

Esta é a fase **B1** do plano (`blend/doc/PLANO_EDICAO_E_CHAT.md`, secção 4). O que ainda não
existe está em [Limites conhecidos](#limites-conhecidos).

## Instalação

Precisa de Python 3.10 ou superior.

```bash
cd blend/client
python -m venv .venv && source .venv/bin/activate      # opcional, mas recomendado
pip install -r requirements.txt                         # PySide6
pip install keyring                                     # opcional: chaves no porta-chaves do sistema
```

## Arrancar

1. Inicia o editor com a API ligada (porta 7420 por omissão):

   ```bash
   radion_blender --api
   ```

2. Inicia o chat (a partir de `blend/client`):

   ```bash
   python -m radion_chat
   ```

A barra de estado mostra `● Editor API connected` quando o editor responde. Se o editor não
estiver a correr, mostra `○ Cannot reach the Radion editor API ... Start the editor with
`radion_blender --api``. O estado é verificado de 2 em 2 segundos.

Na primeira vez não há perfis: clica em **New**, escolhe um exemplo (Ollama, DeepSeek, OpenAI),
completa os campos e grava.

## Perfis de LLM

Um perfil diz **a que servidor e modelo** falar. Usa o protocolo OpenAI-compatível
(`POST <base_url>/chat/completions` com `tools`), por isso serve qualquer servidor que o fale.
O `base_url` é usado tal como o escreves: inclui o prefixo de versão se o servidor o exigir.

| Campo | Significado |
|---|---|
| `name` | nome no seletor |
| `base_url` | endereço do servidor, p. ex. `http://localhost:11434/v1` |
| `model` | nome do modelo tal como o servidor o conhece |
| `api_key_env` | **nome** da variável de ambiente com a chave (não a chave) |
| `vision` | marca se o modelo aceita imagens. **Não é adivinhado**: és tu que o dizes |
| `simplify_schema` | simplifica os esquemas das tools (ver abaixo); liga se o servidor rejeitar as tools |
| `stream` | resposta em *streaming* (SSE); se o servidor recusar, o cliente repete sem *streaming* |
| `temperature` | vazio = valor por omissão do servidor |
| `max_steps` | máximo de chamadas ao modelo por pedido (40) |
| `request_timeout` | segundos sem receber nada do servidor antes de desistir (300) |
| `api_url` | onde está a API do editor (`http://127.0.0.1:7420`) |
| `context_chars` | tamanho máximo do histórico enviado, em caracteres (120 000) |
| `system_prompt_extra` | texto extra acrescentado ao prompt de sistema |

### Exemplo: Ollama (local)

```json
{
  "name": "ollama",
  "base_url": "http://localhost:11434/v1",
  "model": "<um modelo que tenhas feito pull e que suporte tools>",
  "api_key_env": "",
  "vision": false
}
```

Sem chave. **Nem todos os modelos locais suportam *tool calling*** — alguns ignoram as tools e
respondem só em texto. Isto não foi verificado aqui para nenhum modelo em concreto: se o
modelo não chamar comandos, experimenta outro, ou ativa `simplify_schema`. Mete `vision: true`
só se o modelo aceitar imagens.

### Exemplo: DeepSeek

```json
{
  "name": "deepseek",
  "base_url": "https://api.deepseek.com",
  "model": "deepseek-chat",
  "api_key_env": "DEEPSEEK_API_KEY",
  "vision": false
}
```

```bash
export DEEPSEEK_API_KEY=...      # antes de iniciar o chat
```

O nome do modelo e a compatibilidade com *tools* e `oneOf` no esquema devem ser confirmados na
documentação atual do serviço (estas coisas mudam); se as tools forem rejeitadas, liga
`simplify_schema`.

Os perfis ficam num ficheiro JSON na pasta de configuração do utilizador
(`~/.config/radion_chat/profiles.json` em Linux, `%APPDATA%\radion_chat\` em Windows,
`~/Library/Application Support/radion_chat/` em macOS). Esse ficheiro **não tem campo para
chaves**.

## Chaves e token

As chaves nunca são escritas em ficheiro. Para cada perfil, a chave é procurada por esta
ordem:

1. variável de ambiente indicada em `api_key_env`;
2. porta-chaves do sistema, se o pacote opcional `keyring` estiver instalado e tiver a chave
   (a caixa "Remember in the system keyring" do editor de perfis guarda-a lá);
3. valor escrito no campo "API key" do editor de perfis: fica **só em memória** e perde-se ao
   fechar a aplicação.

O editor de perfis mostra de onde veio a chave encontrada. O mesmo vale para o token da API do
editor (`RADION_BLENDER_API_TOKEN`, ou campo em **API settings...**).

## Confirmações

Estes comandos escrevem ficheiros ou deitam fora trabalho, por isso o cliente pede confirmação
(janela **Allow / Deny**, com os argumentos) antes de os executar:

`save_mesh`, `export_obj`, `export_gltf`, `new_document`, `load_mesh`

Se recusares, o modelo é informado de que não deve repetir o comando. Podes desligar isto em
**API settings...** ("Ask before save, export, load and new-document commands"). O prompt de
sistema também diz ao modelo para não os chamar sem que o utilizador o peça.

## Usar

- **Enter** envia, **Shift+Enter** muda de linha.
- **Stop** (o botão Send passa a Stop) interrompe já: também a meio da resposta em *streaming*.
- Cada chamada a um comando aparece como uma entrada que se abre com o clique: comando,
  argumentos, resultado e erros.
- O painel da direita mostra o último *screenshot*; as miniaturas por baixo são os anteriores
  (clica para ampliar). Sem `vision`, os *screenshots* aparecem aqui mas não são enviados ao
  modelo (o cliente avisa uma vez).
- **Undo this request** desfaz as edições que o pedido mais recente fez no editor (ver abaixo).
- **New chat** limpa a conversa. **Save conversation...** grava-a em JSON (sem as imagens).

### "Undo this request"

Chama o comando `undo` com o número de passos de desfazer que o pedido acrescentou. Nem todos
os comandos com `readOnly: false` são passos de desfazer no editor: `select`,
`set_part_visible`, `set_animation` e os que gravam ficheiros (`save_mesh`, `export_obj`,
`export_gltf`) não são (verificado contra o editor real), e um comando que falhou não mudou
nada. O cliente conta só os que são (lista `NOT_UNDO_STEPS` em `radion_chat/agent.py`), para
nunca desfazer o teu trabalho anterior. `new_document` e `load_mesh` esvaziam o histórico de
desfazer do editor, por isso só se desfaz o que veio depois deles. Se editares o modelo à mão no
editor durante um pedido, a contagem deixa de ser exata.

## Como funciona

- As **tools** vêm de `GET /api/commands` a cada pedido: um comando novo no editor aparece no
  chat sem alterar o cliente.
- **`simplify_schema`** troca `oneOf`/`anyOf` por uma descrição em texto ("Accepts a number or
  an array of exactly 3 numbers") e remove `minItems`, `maxItems`, `additionalProperties`,
  `format`, `default`, etc., para servidores que engasgam com esses campos.
- O **histórico é limitado**: só os 2 últimos *screenshots* ficam anexados, resultados antigos
  e grandes (p. ex. `get_mesh_data`) são cortados, e se mesmo assim passar `context_chars` os
  pedidos mais antigos são descartados.
- Os erros da API (`invalid_params`, `failed`, ...) voltam ao modelo em texto claro, para ele se
  corrigir. Argumentos de tool que não sejam JSON válido também: o cliente não os executa e
  diz ao modelo para reenviar.
- O ciclo pára quando o modelo responde em texto, ao fim de `max_steps`, ou com Stop.
- Com OpenAI-compatível não há imagens dentro de mensagens `tool`: com `vision: true` o cliente
  acrescenta a seguir uma mensagem `user` com o *screenshot* (data URI); com `vision: false`
  envia só texto.

## Testes

```bash
pip install -r requirements-dev.txt
cd blend/client
QT_QPA_PLATFORM=offscreen python -m pytest
```

Não precisam de rede, de GPU nem de LLM: um servidor OpenAI-compatível falso e um servidor
falso da API do editor (`tests/fake_llm_server.py`, `tests/fake_radion_api.py`) correm em
*threads* locais. A lista de comandos dos testes é a real (`tests/fixtures/commands.json`,
capturada do editor).

Teste de ponta a ponta contra o editor real (ignorado se não indicares um editor):

```bash
# com um editor já a correr
RADION_E2E_API_URL=http://127.0.0.1:7420 python -m pytest tests/test_e2e_editor.py
# ou deixando o teste arrancá-lo em headless (precisa de xvfb-run)
RADION_EDITOR_BIN=/caminho/para/radion_blender python -m pytest tests/test_e2e_editor.py
```

O teste usa um LLM falso com chamadas de comandos pré-definidas; **nenhum LLM real foi
testado** até agora.

## Limites conhecidos

- **Só há um fornecedor**: OpenAI-compatível. Anthropic nativo, o modo "JSON em texto" para
  modelos sem *tool calling* e o fornecedor MCP estão planeados para B2/B3.
- O suporte a *tools* varia por modelo e por versão do servidor e **não foi verificado** com
  Ollama, DeepSeek ou outros: alguns modelos locais não chamam tools.
- Os pedidos ao LLM não passam por proxies HTTP(S) configurados no sistema (usam ligação
  direta); a API do editor também nunca passa por proxy.
- A contagem de "Undo this request" parte de uma tabela de comandos (ver acima); quando o
  editor passar a indicar na listagem se um comando é desfazível (`undoable`), o cliente já a
  usa.
- Comandos que dão *timeout* no editor (504, p. ex. um diálogo aberto) não contam como edições,
  mesmo que tenham chegado a correr.
- A interface está em inglês.

## Resolução de problemas

| Sintoma | O que fazer |
|---|---|
| `○ Cannot reach the Radion editor API` | inicia o editor com `radion_blender --api` (ou confirma a porta em **API settings...**) |
| erro HTTP 401 do editor | o editor tem token: define `RADION_BLENDER_API_TOKEN` ou preenche-o em **API settings...** |
| `Cannot reach the LLM server` | confirma o `base_url` do perfil e que o servidor (p. ex. Ollama) está a correr |
| HTTP 401/403 do servidor de LLM | falta a chave, ou a variável de `api_key_env` não está definida no terminal que iniciou o chat |
| HTTP 400 a falar de `tools` ou do esquema | liga `simplify_schema` no perfil; se continuar, o modelo pode não suportar tools |
| o modelo só responde em texto e não constrói nada | esse modelo provavelmente não faz *tool calling*; usa outro |
| "Stopped after N steps" | o limite `max_steps` foi atingido; envia outra mensagem para continuar, ou aumenta o limite |
| a resposta demora muito a começar | modelos locais podem levar minutos a carregar; `request_timeout` define quanto esperar por bytes |

## Estrutura

```
radion_chat/
  api_client.py   cliente REST da API do editor (urllib)
  tools.py        /api/commands -> tools OpenAI; simplify_schema
  agent.py        ciclo do agente, confirmações, contagem para o undo
  history.py      limites do histórico
  prompts.py      prompt de sistema
  profiles.py     perfis e definições (JSON); secrets.py: chaves
  session.py      constrói um Agent a partir de um perfil
  llm/            LlmProvider; openai_compat.py + openai_wire.py
  ui/             janela Qt (PySide6): chat, painel de imagens, diálogos, threads
tests/            pytest, servidores falsos, teste de ponta a ponta
```
