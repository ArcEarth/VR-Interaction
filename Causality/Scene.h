#pragma once
#include "Common\Renderable.h"
#include "Interactive.h"
namespace DirectX
{
	namespace Scene
	{
		interface IScene abstract: public IRenderable , public IDeviceNotify
		{
		public:
			virtual ~IScene();

		};
	}

}
