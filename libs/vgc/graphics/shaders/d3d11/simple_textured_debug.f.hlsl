
struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 clipPos : POSITION0;
    float2 uv : TEXCOORD0;
    float4 col : COLOR0;
};

float avg(float3 x) {
    return (x.x + x.y + x.z) / 3;
}

float4 main(PS_INPUT input)
    : SV_Target {

    const float square_size = 4;
    const float m = square_size * 2;
    float2 uvm = (m + (input.uv % m)) % m;

    if ((uvm.x < square_size) == (uvm.y < square_size)) {
        return float4(0, 0, 0, input.col.a * 2);
    }
    if (input.uv.y < 0) {
        return float4(
            input.col.rgb + ((round(avg(input.col.rgb)) * 2 - 1) * 0.5), input.col.a);
    }

    return input.col;
}
