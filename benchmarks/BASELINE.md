# Baseline de build — antes do port

Esta tabela é a referência para o port de GLM e contentores STL para `mathc`
e `ct`. Cada nova medição deve usar exactamente a mesma configuração, salvo
quando a alteração for anotada na tabela.

## Ambiente

| Campo | Valor |
|---|---|
| Commit | `23b16d708107aed610276003f9dbe361f5be9ff5` |
| Sistema | Linux 7.0.0-30-generic x86_64 |
| CPU | AMD Ryzen 5 5500U (12 CPUs lógicos) |
| Compilador | GCC 13.3.0 |
| CMake / gerador | 3.28.3 / Ninja |
| Configuração | Release, `RADION_ENABLE_SANITIZERS=OFF` |
| Paralelismo | `-j12` |

O alvo é exclusivamente `radion_editor`. Runner, Blender, voxel demo e testes
estão desligados; as bibliotecas necessárias ao editor, ImGui e as outras
dependências transitivas continuam incluídas. Isto mantém a medição focada no
produto que será comparado depois do port.

## Resultados

| Métrica | Baseline GLM + STL | Pós-port mathc + ct | Delta |
|---|---:|---:|---:|
| Configuração CMake | 1,25 s |  |  |
| Build limpo `radion_editor` | 109,43 s |  |  |
| Build incremental `radion_editor` | 0,43 s |  |  |
| Física: box pile 1.024 corpos | 20,129 ms/step (21,053 ms máx.) |  |  |
| Física: eventos, 128 corpos | 2,236 ms/step (152.031 callbacks) |  |  |
| Física: softbody 45×45 | 1,324 ms/step |  |  |
| Física: trimesh 1.002.528 triângulos | build 375,7 ms; 9,037 ms/step |  |  |
| Cena: serialização | 52,563 ms (mediana) |  |  |
| Cena: parse JSON | 13,650 ms (mediana) |  |  |
| Cena: desserialização | 5,975 ms (mediana) |  |  |
| Cena: update/frame | 0,078 ms (mediana) |  |  |

O build limpo configurou `build-baseline-editor` do zero e executou
`cmake --build build-baseline-editor --target radion_editor -j 12`. O
incremental é a repetição imediata sem alterações e serve apenas para detectar
regressões de dependências/rebuilds inesperados; não substitui o build limpo.

O benchmark de cena usa 4.096 objetos numa árvore de oito filhos, produz
914.834 bytes de JSON e valida a contagem de objetos após cada uma das 15
desserializações. O cenário de física foi executado no mesmo build Release.
