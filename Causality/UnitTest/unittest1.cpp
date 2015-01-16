#include "stdafx.h"
#include "CppUnitTest.h"
#include "..\Common\tree.h"
#include <memory>
#include <iostream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace std;
using namespace stree;


namespace UnitTest
{		
	struct test_node : public tree_node<test_node>
	{
		test_node(int id)
			: ID(id)
		{}

		int ID = -1;
	};

	TEST_CLASS(TreeTemplateTest)
	{
	public:

		std::unique_ptr<test_node> root;
		TreeTemplateTest()
		{
			Logger::WriteMessage("In TreeTemplateTest");
			root.reset(new test_node(0));
			auto node1 = new test_node(1);
			node1->append_children_back(new test_node(2));
			node1->append_children_back(new test_node(3));
		}
		
		TEST_METHOD(TestMethod1)
		{
			// TODO: Your test code here
		}

	};
}