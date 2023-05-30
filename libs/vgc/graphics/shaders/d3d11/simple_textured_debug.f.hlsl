
struct PS_INPUT
{
    float4 pos : SV_POSITION;
    float4 clipPos : POSITION0;
    float2 uv : TEXCOORD0;
    float4 col : COLOR0;
};

float avg(float3 x)
{
    return (x.x + x.y + x.z) / 3;
}

float4 main(PS_INPUT input)
    : SV_Target
{

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

    const float square_size = 0.01;
    
    // prevent artifacts due to discontinuities at caps
    input.uv.x += square_size * 0.5;
    input.uv.y *= (1 - square_size * 0.1);
    
    const float m = square_size * 2;
    float2 uvm = (m + (input.uv % m)) % m;

    //if (bool(uvm.x < square_size) != bool(uvm.y < square_size))
    //{
    //    discard;
    //}
    // lerp(input.col.rgb, input.col.gbr, input.uv.x / 160.0 % 0.5);
    float3 c1 = input.col.rgb;
    if (input.uv.y < 0)
    {
        float a = 1 - round(avg(c1));
        c1 = (c1 + float3(a, a, a)) * 0.5;
    }
    return float4(c1, input.col.a * 2);
    
    
}
