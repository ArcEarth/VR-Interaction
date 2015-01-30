#pragma once

namespace std
{
	template <typename T>
	inline void delete_s(T* &pData){
		if (pData) {
			delete pData;
#ifdef _DEBUG
			pData = nullptr;
#endif
		}
	}

	// Becareful with this iterator , 
	template<typename _Ty>
	class tree_node
	{
	public:
		typedef _Ty value_type;
		class const_iterator;
		class iterator
		{
		public:
			typedef iterator self_type;
			typedef tree_node value_type;  
			typedef value_type& reference;  
			typedef value_type* pointer;  
			typedef std::forward_iterator_tag iterator_category;  
			typedef int difference_type;  
		private:
			pointer pNode;
		public:
			iterator (void)
				: pNode(nullptr)
			{}
			explicit iterator (const pointer ptr)
				: pNode(ptr)
			{}

			iterator& operator ++(int){
				if (pNode)
					pNode = pNode->_brother;
				return *this;
			}

			self_type& operator ++(){
				if (pNode->_children) pNode = pNode->_children;
				else if (pNode->_brother) pNode = pNode->_brother;
				else
				{
					while ((pNode->_parent) && (!pNode->_parent->_brother || pNode->_parent->_brother == pNode))
						pNode = pNode->_parent;
					if (pNode->_parent)
						pNode = pNode->_parent->_brother;
					else
						pNode = nullptr;
				}
				return *this;
			}

			//iterator& operator --(){
			//	if (pNode->_parent->_children != pNode)
			//		pNode = pNode->_parent;
			//	return this;
			//}

			reference operator * () {
				return *pNode;
			}
			pointer operator -> (){
				return pNode;
			}

			bool operator == (const self_type rhs) const{
				return pNode == rhs.pNode;
			}

			bool operator != (const self_type rhs) const{
				return pNode != rhs.pNode;
			} 

			tree_node* ptr() {
				return pNode;
			}

			operator const_iterator () const{
				return *this;
			}
		};
		class const_iterator
		{
		public:
			typedef const_iterator self_type;
			typedef tree_node value_type;  
			typedef const value_type& reference;  
			typedef const value_type* pointer;  
			typedef std::forward_iterator_tag iterator_category;  
			typedef int difference_type;  
		private:
			pointer pNode;
		public:
			const_iterator (void)
				: pNode(nullptr)
			{}
			explicit const_iterator (const pointer ptr)
				: pNode(ptr)
			{}

			self_type& operator --(){
				if (pNode)
					pNode = pNode->_brother;
				return *this;
			}
			self_type& operator --(int){
				return --(*this);
			}

			self_type& operator ++(){
				if (pNode->_children) pNode = pNode->_children;
				else if (pNode->_brother) pNode = pNode->_brother;
				else
				{
					while ((pNode->_parent) && (!pNode->_parent->_brother || pNode->_parent->_brother == pNode))
						pNode = pNode->_parent;
					if (pNode->_parent)
						pNode = pNode->_parent->_brother;
					else
						pNode = nullptr;
				}
				return *this;
			}

			self_type& operator ++(int){
				return ++(*this);
			}

			reference operator * () const {
#ifdef _DEBUG
				if (!pNode)
					throw new std::out_of_range("iterator has reach it's end");
#endif
				return *pNode;
			}
			pointer operator -> () const {
#ifdef _DEBUG
				if (!pNode)
					throw new std::out_of_range("iterator has reach it's end");
#endif
				return pNode;
			}

			bool operator == (const self_type rhs) const{
				return pNode == rhs.pNode;
			}

			bool operator != (const self_type rhs) const{
				return pNode != rhs.pNode;
			} 

			pointer ptr() const{
				return pNode;
			}
		};
	public:
		_Ty Entity;
	protected:
		tree_node *_brother;
		tree_node *_children;
		tree_node *_parent;
//		tree_node *_next;

	public:

		tree_node()
			: _parent(nullptr) , _brother(nullptr) , _children(nullptr)
		{
		}
		tree_node(const _Ty& entity)
			: Entity(entity) , _parent(nullptr) , _brother(nullptr) , _children(nullptr)
		{
		}

		~tree_node()
		{
			delete_s(_brother);
			delete_s(_children);
#ifdef _DEBUG
			_parent = nullptr;
#endif
		}

		tree_node(const tree_node& rhs)
			: Entity(rhs.Entity) , _parent(nullptr) , _brother(nullptr) , _children(nullptr)
		{
			rhs.for_all_children([&](const tree_node& SrcNode)
			{
				tree_node *pNode = new tree_node(SrcNode);
				this->_insert_as_child_back(pNode);
			});
		}

		tree_node& operator = (const tree_node& rhs)
		{
			this->Entity = rhs.Entity;
			this->_parent = nullptr;
			this->_brother = nullptr;
			rhs.for_all_children([&](tree_node& SrcNode)
			{
				tree_node *pNode = new tree_node(SrcNode);
				this->Children_Add(pNode);
			});
		}
		const_iterator Children_begin() const{
			return const_iterator(this->_children);
		}

		const_iterator Children_end() const{
			return const_iterator(nullptr);
		}

		const_iterator Subtree_begin() const
		{
			return const_iterator(this);
		}

		const_iterator Subtree_end() const
		{
			auto pNode = this->_children;
			while ((pNode->_parent) && (!pNode->_parent->_brother || pNode->_parent->_brother == pNode))
				pNode = pNode->_parent;
			if (pNode->_parent)
				pNode = pNode->_parent->_brother;
			else
				pNode = nullptr;
			return const_iterator(pNode);
		}

		iterator Children_begin() {
			return iterator(this->_children);
		}

		iterator Children_end() {
			return iterator(nullptr);
		}

		iterator Subtree_begin()
		{
			return iterator(this);
		}

		iterator Subtree_end()
		{
			auto pNode = this->_children;
			while ((pNode->_parent) && (!pNode->_parent->_brother || pNode->_parent->_brother == pNode))
				pNode = pNode->_parent;
			if (pNode->_parent)
				pNode = pNode->_parent->_brother;
			else
				pNode = nullptr;
			return iterator(pNode);
		}

		template <typename _Func>
		inline void for_all_descendants(const _Func & func) const
		{
			auto end = Subtree_end();
			for (auto itr = Children_begin(); itr != end; ++itr)
			{
				func(*itr);
			}
		}

		template <typename _Func>
		inline void for_all_descendants(const _Func & func)
		{
			auto end = Subtree_end();
			for (auto itr = Children_begin(); itr != end; ++itr)
			{
				func(*itr);
			}
		}

		// Make A DFS order visit for all node in sub tree
		template <typename _Func>
		inline void for_all_subtree(const _Func & func) const
		{
			auto end = Subtree_end();
			for (auto itr = Subtree_begin(); itr != end; ++itr)
			{
				func(*itr);
			}
		}

		// Make A DFS order visit for all node in sub tree
		template <typename _Func>
		inline void for_all_subtree(const _Func & func)
		{
			auto end = Subtree_end();
			for (auto itr = Subtree_begin(); itr != end; ++itr)
			{
				func(*itr);
			}
		}

		// Visit all the direct-child
		template <typename _Func>
		inline void for_all_children(const _Func &func) const
		{
			for (auto itr = Children_begin(); itr != Children_end(); itr--)
			{
				func(*itr);
			}
		}

		// Visit all the direct-child
		template <typename _Func>
		inline void for_all_children(const _Func &func)
		{
			for (auto itr = Children_begin(); itr != Children_end(); itr++)
			{
				func(*itr);
			}
		}

		const tree_node* Parent() const{
			tree_node* p = this;
			while (p->_parent && p->_parent->_children != p)
				p=p->_parent;
			return p->_parent;
		}

		tree_node* Parent() {
			tree_node* p = this;
			while (p->_parent && p->_parent->_children != p)
				p=p->_parent;
			return p->_parent;
		}

		inline void Children_Add(tree_node* pRoot)
		{
#ifdef _DEBUG
			if (!pRoot->is_root()) 
				throw new std::exception("insert node isn't a root node!");
#endif
			_insert_as_child_front(pRoot);
		}

		inline void Children_Append(tree_node* pForest)
		{
			_insert_as_child_back(pForest);
		}

		tree_node& operator += (const tree_node *pNode)
		{
			_insert_as_child_back(pNode);
		}

		// use this method to make a sub-tree
		void isolate(){
			if (this->_parent == this)
#ifdef _DEBUG
				throw new std::exception("Can not separate the root tree_node.");
#else
				return;
#endif
			
			if (this->_parent->_children == this){
				this->_parent->_children = this->_brother;
			} else
			{
				this->_parent->_brother = this->_brother;
			}
			this->_parent = this;
			this->_brother = nullptr;
		}

		// permenently delete one child , if not mean this , using this->children->isolate()
		void Child_Remove(tree_node* &pNode)
		{
			if (pNode->Parent()!=this)
#ifdef _DEBUG
				throw new exception("The given Node isn't a child of this.");
#else
				return;
#endif
			pNode->isolate();
			delete_s(pNode);
		}

		bool is_root() const{
			return (!this->_parent);
		}

		void _insert_as_child_back(tree_node* ptr){
			tree_node* p = this->_children;
			if (p==nullptr){
				this->_children = ptr;
				ptr->_parent = this;
			}else{
				while (p->_brother!=nullptr) 
					p = p->_brother;
				p->_brother = ptr;
				ptr->_parent = p;
			}
		}

		void _insert_as_child_front(tree_node* ptr){
			ptr->_brother = this->_children;
			if (ptr->_brother)
				ptr->_brother->_parent = ptr;
			ptr->_parent = this;
			this->_children = ptr;
		}	
	};


	template <typename _Ty,typename _NodeType = tree_node<_Ty>>
	class tree
	{
	public:
		typedef _Ty value_type;
		typedef _NodeType node_type;
		typedef typename node_type::iterator iterator;
		typedef typename node_type::const_iterator const_iterator;
	protected:
		node_type* _root;

	public:
		tree()
			: _root(nullptr)
		{
		}
		tree(const tree& rhs)
			: _root(new node_type(*rhs._root))
		{
		}

		tree& operator = (const tree& rhs){
			clear();
			_root = new node_type(*rhs._root);
			return *this;
		}

		~tree(){
			clear();
		}

		bool empty() const
		{
			return _root==nullptr;
		}

		iterator begin(){
			return iterator(root());
		}

		const_iterator begin() const{
			return const_iterator(_root);
		}

		const_iterator end() const{
			return const_iterator(nullptr);
		}
		iterator end() {
			return iterator(nullptr);
		}

		node_type* root(){
			return _root;
		}

		const node_type* root() const{
			return _root;
		}

		void clear()
		{
			if (_root)
			{
				delete _root;
				_root = nullptr;
			}
		}

		// Define the func like [&](node_type& node) {node = bla.bla.bla...}
		template <typename _Func>
		inline void for_all(const _Func &func)
		{
			for (auto itr = begin(); itr != end(); ++itr)
			{
				func(*itr);
			}
		}

		template <typename _Func>
		inline void for_all(const _Func &func) const
		{
			for (auto itr = begin(); itr != end(); ++itr)
			{
				func(*itr);
			}
		}
	};

	template <typename T>
	struct hierachy_node
	{
		T *Parent;
		std::list<T*> Children;
	};

	template <typename _Tentity>
	class tree_node_inhert
		: hierachy_node<tree_node_inhert<_Tentity>>
		, _Tentity
	{
	public:
		tree_node_inhert();
		~tree_node_inhert();
	};
}