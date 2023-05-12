
cbuffer vertexBuffer : register(b0) {
    float4x4 projMatrix;
    float4x4 viewMatrix;
    float4 viewport;
    unsigned int frameStartTimeInMs;
};

struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 clipPos : POSITION0;
    float4 col : COLOR0;
};

float4 main(PS_INPUT input)
    : SV_Target {

    float3 c = input.col.rgb;

    // gradient in V (-1:red, 0: black, 1:green) with 0.5 opacity
    //if (input.uv.y > 0) {
    //    return float4(input.uv.y, 0.f, 0.f, 0.5f);
    //}
    //else {
    //    return float4(0.f, 0.f, -input.uv.y, 0.5f);
    //}

    // smooth brush test (easings.net/#easeInOutQuad)
    //float x = 1.f - abs(input.uv.y);
    //float k = -2 * x + 2;
    //float ca = x < 0.5 ? 2 * x * x : 1 - k * k / 2;
    //return float4(input.col.rgb, input.col.a * ca);

    float t = frameStartTimeInMs / 4000.f;
    t = (1.f + (t % 1.f)) % 1.f;

    float2 dir = normalize(float2(1.0, 1.0));
    float y = dot(input.clipPos.xy - float2(-0.5, -0.5), dir) - t;

    //    const float square_size = 0.05;
    //const float m = square_size * 2;
    //float2 uvm = (m + ((input.uv + 2.0) % m)) % m;
    //
    //if ((uvm.x < square_size) == (uvm.y < square_size)) {
    //    return float4(0, 0, 0, input.col.a * 2);
    //}
    //float3 c =
    //    input.col.rgb; // lerp(input.col.rgb, input.col.gbr, input.uv.x / 160.0 % 0.5);
    //if (input.uv.y < -0.1) {
    //    c += (round(avg(c)) * 2 - 1) * 0.5;
    //}

    return float4(c.r, y, c.b, input.col.a);
}
