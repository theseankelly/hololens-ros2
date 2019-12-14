#pragma once
#include "Node.g.h"

using namespace winrt::Windows::Foundation;

namespace winrt::HoloLensRos2::implementation
{
	class Node : public NodeT<Node>
	{
	public:
		explicit Node(hstring const& name);
		
		IAsyncAction StartPublishing();

	private:
		hstring node_name_;
	};
}
namespace winrt::HoloLensRos2::factory_implementation
{
	struct Node : NodeT<Node, implementation::Node>
	{
	};
}
