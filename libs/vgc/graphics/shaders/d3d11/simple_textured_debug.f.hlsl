
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
    // smooth brush test (easings.net/#easeInOutQuad)
    // v: [-1, 1]
    float v = input.uv.y;
    // u: [0, 1]
    float u = input.uv.x;
    float vAbs = abs(v);

    // x: linear, 1 on centerline, 0 on offset line
    float x = 1.f - vAbs;

    // vGrad: easeInOut, 1 on centerline, 0 on offset line
    float k = -2 * x + 2;
    float gradV = x < 0.5 ? 2 * x * x : 1 - k * k / 2;
   
    float light = 0.9 * gradV + (vAbs < 0.32 ? 0.1 : 0);

    float t0 = 0.1;
    float t1 = 0.25;
    float3 color = lerp(input.col.rgb, input.col.gbr, input.uv.x * 0.5);
    float colorBurn = 0.0;
    
    float minCC = min(min(color.r, color.g), color.b);
    float burn = max(0, 0.9f - minCC);
    if (vAbs < t1)
    {
        if (vAbs > t0)
        {
            float cx = 1.f - (vAbs - t0) / (t1 - t0);
            float ck = -2 * cx + 2;
            float grad1 = cx < 0.5 ? 2 * cx * cx : 1 - ck * ck / 2;
            colorBurn = grad1 * burn;
        }
        else
        {
            colorBurn = burn;
        }
    }

    return float4(color + colorBurn, input.col.a * light);
}
