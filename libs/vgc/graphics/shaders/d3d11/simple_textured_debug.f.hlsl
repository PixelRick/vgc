
cbuffer vertexBuffer : register(b0) {
    float4x4 projMatrix;
    float4x4 viewMatrix;
    float4 viewport;
    unsigned int frameStartTimeInMs;
};

struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 clipPos : POSITION0;
    float2 uv : TEXCOORD0;
    float2 pt : POSITION1;
    float4 col : COLOR0;
};

float hash(in float t) {
    return frac((sin(t) + 1.0) * 214748.3647);
}

// Perlin's new interpolation function, which is a Hermite polynomial
// of degree five and results in a C2 continuous noise function
float2 fade(in float2 t) {
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}
float fade(in float t) {
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

float noise(in float2 t) {
    float2 p = floor(t);
    float2 f = frac(t);
    f = fade(f);

    float s = p.x + 123.0 * p.y;
    return lerp(
        lerp(hash(s), hash(s + 1.0), f.x),
        lerp(hash(s + 123.0), hash(s + 124.0), f.x),
        f.y);
}

float avg(float3 x) {
    return (x.x + x.y + x.z) / 3;
}

float4 main(PS_INPUT input)
    : SV_Target {

    // gradient in V (-1:red, 0: black, 1:green) with 0.5 opacity
    //if (input.uv.y > 0) {
    //    return float4(input.uv.y, 0.f, 0.f, 0.5f);
    //}
    //else {
    //    return float4(0.f, 0.f, -input.uv.y, 0.5f);
    //}

    // perlin test
    float n = noise(input.pt);

    // smooth brush test (easings.net/#easeInOutQuad)
    float x = 1.f - abs(input.uv.y);
    float k = -2 * x + 2;
    float ca = x < 0.5 ? 2 * x * x : 1 - k * k / 2;
    if (ca * 1.2 < n) {
        discard;
    }

    float3 c1 = input.col.rgb;
    float3 c2 = input.col.gbr;

    return float4(lerp(c1, c2, input.uv.x) * lerp(0.9, 1, n), input.col.a * ca);

    const float square_size = 0.05;
    const float m = square_size * 2;
    float2 uvm = (m + ((input.uv + 2.0) % m)) % m;

    if ((uvm.x < square_size) == (uvm.y < square_size)) {
        return float4(0, 0, 0, input.col.a * 2);
    }
    float3 c =
        input.col.rgb; // lerp(input.col.rgb, input.col.gbr, input.uv.x / 160.0 % 0.5);
    if (input.uv.y < -0.1) {
        c += (round(avg(c)) * 2 - 1) * 0.5;
    }

    return float4(c, input.col.a);
}
