#version 450 core
 
in vec2  vUV;
in vec3  vNormal;
in vec3  vWorldPos;
in float vAltura01;
in vec3  vTint;
in vec2  vMotionNDC;

#include "grass_uniforms.glsl"
layout(binding = 0) uniform sampler2D uGrassTex;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec2 FragVelocity;
layout(location = 2) out float FragReactive;

void main()
{
    vec4 tex = texture(uGrassTex, vUV);

    
    float a = tex.a;
    if (uModo == 0)
    {
        if (a < uAlphaCut) discard;
    }
    else
    {
        // Ja' desenhado pela passagem opaca, ou vazio de mais para valer o
        // custo de misturar.
        if (a >= uAlphaCut || a < 0.03) discard;
    }

    vec3 N = normalize(vNormal);
    vec3 L = normalize(-uLightDir);
    vec3 V = normalize(uViewPos - vWorldPos);

    vec3 albedo = tex.rgb * vTint;

    // Difuso "meio-lambert": em vez de max(dot,0), remapeia-se -1..1 para
    // 0..1. Erva iluminada de lado nao tem um lado completamente preto --
    // a luz atravessa e espalha-se pelas folhas.
    float ndl = dot(N, L) * 0.5 + 0.5;
    ndl *= ndl;

    // Translucencia: quando a folha esta ENTRE nos e o sol, ela acende. E' o
    // efeito mais caracteristico da vegetacao ao contraluz, e sem ele a erva
    // le'-se como plastico.
    float trans = pow(max(dot(V, -L), 0.0), 4.0) * 0.6;

    // A base do tufo recebe menos luz -- oclusao propria. Sem isto o tufo
    // parece flutuar, porque a raiz fica tao clara como as pontas.
    float ao = mix(0.45, 1.0, vAltura01);

    vec3 cor = albedo * (uLightColor * (ndl + trans) + uAmbient) * ao;

    // Na passagem do contorno o alfa da textura vira opacidade, normalizado
    // para o intervalo abaixo do corte -- assim a franja desvanece de 0 a 1 em
    // vez de aparecer de repente.
    float alfa = (uModo == 0) ? 1.0 : smoothstep(0.03, uAlphaCut, a);
    FragColor = vec4(cor, alfa);
    FragVelocity = vMotionNDC;
    // A passagem opaca resolve-se pelo historico como qualquer geometria. A
    // franja e' misturada, e a mistura aplica-se a todos os anexos: escrever
    // 1.0 aqui deixa o proprio alfa decidir quanto de reactivo fica no pixel,
    // que e' exactamente a semantica desejada -- quanto mais a franja cobre,
    // menos historico se aproveita.
    FragReactive = (uModo == 0) ? 0.0 : 1.0;
}
