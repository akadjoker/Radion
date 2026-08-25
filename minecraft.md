 o `Noise` certo para isto: `Noise::Perlin` 3D com seed determinística e `Noise::Voronoi`. O `Landscape` já o usa, portanto podemos reutilizar a mesma base sem acoplar o mundo voxel ao componente `Landscape`.

E não vamos limitar o sistema a ar/terra/pedra. Vamos criar um `BlockRegistry` extensível; o primeiro mundo terá um conjunto pequeno para validar o fluxo, mas poderá aceitar qualquer bloco definido por dados.

Estrutura aprovada:

```text
runtime/voxel/
    CMakeLists.txt
    include/VoxelBlock.h
    include/VoxelChunk.h
    include/VoxelWorld.h
    include/VoxelMesher.h
    include/VoxelRaycast.h
    src/VoxelChunk.cpp
    src/VoxelWorld.cpp
    src/VoxelMesher.cpp
    src/VoxelRaycast.cpp

tests/VoxelTests.cpp
```

`radion_voxel` deve depender de `radion_render` para produzir `MeshData`, mas não deve conhecer `Scene`, `GameObject` ou o editor. Assim preservamos a arquitetura em camadas.

Plano de implementação:

1. Fundação do módulo

- Criar a biblioteca `radion_voxel` e adicioná-la ao CMake.
- Definir `BlockId` como `u16`.
- Criar `BlockDefinition`: sólido, transparente, corta luz, emite luz, material/atlas por face, tipo de renderização.
- Criar `BlockRegistry`, sem IDs ou materiais hardcoded no mesher.
- Criar `VoxelChunk` de `32×32×32`, com conversão segura de coordenadas locais/globais.
- Criar `VoxelWorld`, um mapa de `ChunkCoord -> VoxelChunk`.

Blocos iniciais para o protótipo: ar, relva, terra, pedra, areia, água, bedrock, tronco, folhas e minério. Tochas entram quando começarmos a luz emissiva.

2. Geração procedural determinística

Usaremos sempre coordenadas mundiais, nunca a posição local do chunk; isso impede falhas nas fronteiras.

- `Perlin` 2D (`x,z`) para altura continental e detalhe.
- `Voronoi` para biomas.
- `Perlin` 3D (`x,y,z`) para cavernas.
- Regras por altura/bioma para relva, areia, pedra, minérios, árvores e água.
- Um `seed` único reproduz exactamente o mesmo mundo em qualquer máquina.

3. Raycast e edição de blocos

- Implementar raycast DDA: percorre os blocos que um raio atravessa, sem depender da mesh.
- O resultado inclui bloco atingido, face atingida, bloco anterior vazio e distância.
- `VoxelWorld::setBlock()` marca o chunk como sujo.
- Se a alteração estiver numa fronteira, marca também o chunk vizinho como sujo.

4. Meshing e renderização

- Começar por gerar somente as faces expostas.
- Gerar submeshes separados para:
  - opacos;
  - recorte por alpha, como folhas;
  - transparentes, como água.
- Usar atlas de texturas e UVs por face.
- Aplicar ambient occlusion por vértice.
- Primeira versão: meshing normal por faces.
- Segunda versão: greedy meshing para unir faces iguais e reduzir drasticamente triângulos.

O mesher entrega `MeshData`; o adaptador de renderização cria/substitui a mesh dinâmica do chunk. Não haverá um draw call por bloco, apenas por chunk visível.

5. Jogador e colisão

- Primeiro, cada chunk sólido gera também uma malha de colisão.
- Reutilizamos `TriangleOctree` e o `CharacterController` já existentes.
- Ao editar um bloco, reconstruímos a colisão apenas do chunk afectado.
- Mais tarde podemos optimizar para colisão AABB directamente na grelha voxel.

6. Mundo jogável local

- Carregar chunks num raio à volta do jogador.
- Gerar chunks novos por seed.
- Descarregar chunks distantes.
- Fazer a geração e meshing no CPU em worker threads; upload OpenGL continua na thread de renderização.
- O engine ainda não parece ter um job system genérico, portanto esta fase inclui uma fila de jobs pequena e explícita para voxels.

7. Persistência, iluminação e escala

- Guardar apenas alterações ao mundo gerado pela seed, comprimidas por chunk/região.
- Luz solar e luz de blocos propagadas na grelha.
- Água/lava e blocos emissivos.
- Culling, prioridades por distância e orçamento de reconstruções por frame.
- Depois: `glMultiDrawIndirect`/buffer arena se a quantidade de chunks justificar.

Testes obrigatórios desde o início:

- Coordenadas mundo ↔ chunk ↔ bloco local.
- Chunks limítrofes e alterações nas fronteiras.
- Geração igual para a mesma seed.
- Raycast DDA.
- Mesher: nenhum triângulo entre dois blocos sólidos.
- Faces correctas para blocos transparentes.
- Mundo e chunk vazios/cheios.

A primeira entrega concreta será: módulo compilável + `BlockRegistry` + chunks 32³ + geração de relva/terra/pedra/bedrock/água usando o `Noise` existente + testes sem OpenGL. Depois partimos para raycast e meshing.