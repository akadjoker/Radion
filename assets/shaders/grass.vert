#version 450 core

 

struct Clump
{
    vec3 position;   float scale;
    vec3 normal;     float rotation;
    float rect;      float _p0, _p1, _p2;
};

// Porte do HairParticleAtlasRect (ShaderInterop_HairParticle.h:25).
struct AtlasRect
{
    vec4  texMulAdd;   // xy = escala, zw = deslocamento
    float size;        // multiplica a altura do tufo
    float aspect;      // largura/altura da regiao
    float _q0, _q1;
};

layout(std430, binding = 4) readonly buffer ClumpBuffer   { Clump clumps[]; };
layout(std430, binding = 5) readonly buffer VisibleBuffer { uint visiveis[]; };
layout(std430, binding = 7) readonly buffer RectBuffer    { AtlasRect rects[]; };

 
struct Sim { vec4 currentTail; vec4 prevTail; };
layout(std430, binding = 8) readonly buffer SimBuffer { Sim sims[]; };

#include "grass_uniforms.glsl"


out vec2  vUV;

out vec3  vNormal;
out vec3  vWorldPos;
out float vAltura01;
out vec3  vTint;
out vec2  vMotionNDC;

float Hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }

void main()
{
    Clump cl = clumps[visiveis[gl_InstanceID]];

    vec3 raiz = cl.position;
    // A semente vem da POSICAO: o mesmo tufo da' sempre a mesma variacao, sem
    // ser preciso guardar nada por tufo.
    vec2 semente = raiz.xz + vec2(raiz.y * 3.7);

    float dist = length(raiz - uCameraPos);
    float fade = 1.0 - smoothstep(uDistMax * 0.7, uDistMax, dist);

    // ---- Geometria: 18 vertices = 3 quads de 6 ----
    int vid = gl_VertexID;
    int quad = vid / 6;
    int i = vid % 6;

    // Dois triangulos por quad: (0,1,2) e (2,1,3).
    // Cantos: 0=(-1,0) 1=(+1,0) 2=(-1,1) 3=(+1,1) -- x lateral, y altura
    int c = (i == 0) ? 0 : (i == 1) ? 1 : (i == 2) ? 2 : (i == 3) ? 2 : (i == 4) ? 1 : 3;
    vec2 canto = vec2(float(c & 1) * 2.0 - 1.0, float(c >> 1));

    float alturaT = canto.y;
    vAltura01 = alturaT;
    // A regiao do atlas manda nas proporcoes: 'size' na altura e 'aspect' na
    // largura. Sem isto, uma haste fina e uma moita larga sairiam ambas
    // quadradas e esticadas -- e' este par que faz cada variante ter a forma
    // que tem na textura.
    AtlasRect rc = rects[clamp(int(cl.rect), 0, uRectCount - 1)];

    // UV dentro da regiao do atlas. v invertido: o topo da textura e' o topo
    // da erva.  uv = uv * texMulAdd.xy + texMulAdd.zw   (o mad dele)
    vec2 uv0 = vec2(canto.x * 0.5 + 0.5, 1.0 - alturaT);
    vUV = uv0 * rc.texMulAdd.xy + rc.texMulAdd.zw;

    float altura  = uAltura * cl.scale * rc.size * fade;
    float largura = altura * rc.aspect * 0.5 * uLargura;
 
    vec3 up = normalize(cl.normal);
    vec3 ref = (abs(up.y) < 0.99) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 t = normalize(cross(ref, up));
    vec3 b = cross(up, t);

    // Os 3 quads a 60 graus, mais a rotacao do proprio tufo para tufos
    // vizinhos nao ficarem alinhados entre si.
    float ang = cl.rotation + float(quad) * (3.14159265 / 3.0);
    vec3 lado = t * cos(ang) + b * sin(ang);

    
    Sim sim = sims[visiveis[gl_InstanceID]];
    vec3 ponta = sim.currentTail.xyz;
    vec3 desvio = ponta - (raiz + up * altura);   // afastamento face ao repouso
    vec3 curvatura = desvio * (alturaT * alturaT);

    vec3 baseP = raiz
               + up * (alturaT * altura)
               + lado * (canto.x * largura);
    vec3 pos = baseP + curvatura;

    float aPique = 1.0 - abs(dot(uCameraUp, up));
    pos += uCameraUp * (aPique * uBendCamera * alturaT * altura);

    // A ponta do frame anterior ja' esta' no buffer de simulacao, por isso a
    // posicao anterior sai da mesma formula com o outro estado. Sem isto o
    // historico do TAA fica preso a' posicao de repouso e a erva ao vento
    // arrasta.
    vec3 desvioPrev = sim.prevTail.xyz - (raiz + up * altura);
    vec3 posPrev = baseP + desvioPrev * (alturaT * alturaT)
                 + uCameraUp * (aPique * uBendCamera * alturaT * altura);

    if (fade <= 0.001)
    {
        pos = raiz - up * 1000.0;   // colapsa fora de alcance
        posPrev = pos;
    }

    vWorldPos = pos;

    // Normal do quad, puxada para a da superficie. Erva real dispersa muita
    // luz; uma normal puramente lateral escurece o tufo e ele fica um recorte
    // preto contra o ceu.
    vec3 nQuad = normalize(cross(lado, up));
    vNormal = normalize(mix(nQuad, up, 0.6));

    // Variacao de tinta SEM clarear: o intervalo antigo ia ate' (1.05,1.0,0.8),
    // ou seja multiplicava a textura por mais de 1 e lavava-a. Agora so'
    // escurece e muda o matiz entre verde-fresco e verde-seco.
    vTint = mix(vec3(0.62, 0.78, 0.42), vec3(0.95, 0.92, 0.60), Hash(semente + 55.3));

    gl_Position = uViewProj * vec4(pos, 1.0);
    vec4 actual = uViewProjNoJitter * vec4(pos, 1.0);
    vec4 anterior = uPrevViewProjNoJitter * vec4(posPrev, 1.0);
    vMotionNDC = anterior.xy / anterior.w - actual.xy / actual.w;
}
