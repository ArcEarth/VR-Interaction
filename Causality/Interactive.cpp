#include "Interactive.h"
#include "CausalityApplication.h"

Causality::IAppComponent::~IAppComponent()
{
	Unregister();
}

void Causality::IAppComponent::Register()
{
	App::Current()->RegisterComponent(this);
}

void Causality::IAppComponent::Unregister()
{
	App::Current()->UnregisterComponent(this);
}
