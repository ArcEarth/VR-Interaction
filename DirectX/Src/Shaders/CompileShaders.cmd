@echo off
rem THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
rem ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
rem THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
rem PARTICULAR PURPOSE.
rem
rem Copyright (c) Microsoft Corporation. All rights reserved.

setlocal
set error=0
set platform=windows
set ShaderModel=5_0

if %2.==xbox. goto continuexbox
if %2.==windows. goto continue

if %2.==debug. (
set exoptions=/Zi /Od
) else (
set exoptions=/Qstrip_reflect /Qstrip_debug 
)

if %3.==xbox. goto continuexbox
if %3.==windows. goto continue

echo usage: CompileShaders target [debug] [xbox | windows]
exit /b

:continuexbox
if not exist "%DurangoXDK%xdk\FXC\amd64\FXC.exe" goto needxdk
set platform=xbox

:continue
goto %1%

:ShadowMapGen
call :CompileShaderHLSL%platform% ShadowMapGen vs VS_NoBone
call :CompileShaderHLSL%platform% ShadowMapGen vs VS_OneBone
call :CompileShaderHLSL%platform% ShadowMapGen vs VS_TwoBone
call :CompileShaderHLSL%platform% ShadowMapGen vs VS_FourBone

call :CompileShaderHLSL%platform% ShadowMapGen ps PS
goto finish

:ShadowMapEffectVS
call :CompileShaderHLSL%platform% ShadowMapEffectVS vs VS_OneLightNoBoneNoTex
call :CompileShaderHLSL%platform% ShadowMapEffectVS vs VS_OneLightNoBoneTex
call :CompileShaderHLSL%platform% ShadowMapEffectVS vs VS_OneLightFourBoneNoTex
call :CompileShaderHLSL%platform% ShadowMapEffectVS vs VS_OneLightFourBoneTex
goto finish

:ShadowMapEffectPS
call :CompileShaderHLSL%platform% ShadowMapEffectPS ps PS_OneLightNoTex
call :CompileShaderHLSL%platform% ShadowMapEffectPS ps PS_OneLightTex
goto finish

:finish
echo.

if %error% == 0 (
    echo Shaders compiled ok
) else (
    echo There were shader compilation errors!
)

endlocal
exit /b

:CompileShaderwindows
set fxc=fxc /nologo %1.fx /T%2_%ShaderModel% /Zpc %exoptions% /E%3 /Fh%platform%\%1_%3.inc /Vn%1_%3
echo.
echo %fxc%
%fxc% || set error=1
exit /b

:CompileShaderHLSLwindows
set fxc=fxc /nologo %1.hlsl /T%2_%ShaderModel% /Zpc %exoptions% /E%3 /Fh%platform%\%1_%3.inc /Vn%1_%3
echo.
echo %fxc%
%fxc% || set error=1
exit /b

:CompileShaderxbox
set fxc="%DurangoXDK%\xdk\FXC\amd64\FXC.exe" /nologo %1.fx /T%2_%ShaderModel% /Zpc %exoptions% /D__XBOX_DISABLE_SHADER_NAME_EMPLACEMENT /E%3 /Fh%platform%\%1_%3.inc /Vn%1_%3
echo.
echo %fxc%
%fxc% || set error=1
exit /b

:CompileShaderHLSLxbox
set fxc="%DurangoXDK%\xdk\FXC\amd64\FXC.exe" /nologo %1.hlsl /T%2_%ShaderModel% /Zpc %exoptions% /D__XBOX_DISABLE_SHADER_NAME_EMPLACEMENT /E%3 /Fh%platform%\%1_%3.inc /Vn%1_%3
echo.
echo %fxc%
%fxc% || set error=1
exit /b

:needxdk
echo ERROR: CompileShaders xbox requires the Microsoft Xbox One XDK