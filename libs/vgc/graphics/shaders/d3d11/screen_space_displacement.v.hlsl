
cbuffer vertexBuffer : register(b0) {
    float4x4 projMatrix;
    float4x4 viewMatrix;
    float4 viewport;
    unsigned int frameStartTimeInMs;
};

struct VS_INPUT {
    float2 pos : POSITION0;
    float2 disp : DISPLACEMENT;
    float4 col : COLOR0;
    float2 ipos : POSITION1;
    uint vid : SV_VertexID;
    uint iid : SV_InstanceID;
};

struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 clipPos : POSITION0;
    float4 col : COLOR0;
};

PS_INPUT main(VS_INPUT input) {
    PS_INPUT output;
    float4 viewPos = mul(viewMatrix, float4(input.pos + input.ipos, 0.f, 1.f));
    float4 viewDisp = mul(viewMatrix, float4(input.disp, 0.f, 0.f));
    output.pos = mul(projMatrix, viewPos);
    float2 dispDir = normalize(viewDisp).xy;
    float dispMag = length(input.disp);

    float p = (1.0 + cos((float(frameStartTimeInMs) + input.vid) / 300.0f)) * 0.5;
    float animScale = 1.0 + 2.0 * (1.0 + cos((float(frameStartTimeInMs) + input.vid) / 300.0f));
    if (input.iid > 0) {
        animScale = 1.0;
    }

    output.pos.xy += animScale * dispMag * dispDir / float2(0.5 * viewport.z, -0.5 * viewport.w);
    output.clipPos = output.pos;
    output.col = input.col;

    output.col *= 0.5 + 0.5 * p;

    return output;
}
