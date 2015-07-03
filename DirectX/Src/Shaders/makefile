all: ShadowMapGen ShadowMapEffect

ShadowMapGen: ShadowMapGen.hlsl
	CompileShaders ShadowMapGen $(fxconfig)

ShadowMapEffect: ShadowMapEffectVS ShadowMapEffectPS

ShadowMapEffectVS: ShadowMapEffectVS.hlsl
	CompileShaders ShadowMapEffectVS $(fxconfig)

ShadowMapEffectPS: ShadowMapEffectPS.hlsl
	CompileShaders ShadowMapEffectPS $(fxconfig)

ShadowMapEffectVS.hlsl ShadowMapEffectPS.hlsl: ShadowMapEffectStructures.hlsli ShadowMapEffectCBuffer.hlsli

clean:
	rmdir /S /Q windows
	rmdir /S /Q xbox