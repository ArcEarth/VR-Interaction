#pragma once
#include <iterator>
#include <vector>
#include <queue>
#include <stack>
#include <cassert>

namespace stltree
{
	namespace Internal
	{
		template <typename T>
		inline void delete_s(T* &pData) {
			if (pData) {
				delete pData;
#ifdef _DEBUG
				pData = nullptr;
#endif
			}
		}
	}
	// Use and only it as the base of your tree node like:
	// class Ty : public tree_node<Ty>
	template<typename _Ty>
	class tree_node
	{
	public:
		typedef _Ty value_type;
		typedef value_type& reference;
		typedef value_type* pointer;
		typedef value_type const & const_reference;
		typedef value_type const * const_pointer;

	public:
		//_Ty Entity;
	protected:
		// Next sibling node
		_Ty *_sibling;
		// First child node
		_Ty *_child;
		// Physical parent node of this node in data structure
		// May be Logical Parent or Prev Sibling
		_Ty *_parent;
		//		tree_node *_next;

	// Basic Properties
	public:

		tree_node()
			: _parent(nullptr), _sibling(nullptr), _child(nullptr)
		{
		}

		~tree_node()
		{
			Internal::delete_s(_sibling);
			Internal::delete_s(_child);
#ifdef _DEBUG
			_parent = nullptr;
#endif
		}

		tree_node(const tree_node& rhs)
			: Entity(rhs.Entity), _parent(nullptr), _sibling(nullptr), _child(nullptr)
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
			this->_sibling = nullptr;
			rhs.for_all_children([&](tree_node& SrcNode)
			{
				tree_node *pNode = new tree_node(SrcNode);
				this->children_append_front(pNode);
			});
		}
		tree_node& operator = (tree_node&& rhs)
		{
			this->_parent = rhs._parent;
			this->_sibling = rhs._sibling;
		}

		// Logical Parent for this node
		const_pointer parent() const {
			pointer p = this;
			while (p->_parent && p->_parent->_child != p)
				p = p->_parent;
			return p->_parent;
		}
		// Logical Parent for this node
		pointer parent() {
			pointer = this;
			while (p->_parent && p->_parent->_child != p)
				p = p->_parent;
			return p->_parent;
		}

		pointer next_sibling()
		{
			return _sibling;
		}
		const_pointer next_sibling() const
		{
			return _sibling;
		}
		pointer prev_sibling()
		{
			if (this->_parent->_sibling == this)
				return _parent;
			else
				return nullptr;
		}
		const_pointer prev_sibling() const
		{
			if (this->_parent->_sibling == this)
				return _parent;
			else
				return nullptr;
		}

		// check if the node is a LOGICAL root node of a tree
		bool is_root() const {
			return (!this->parent());
		}
		bool is_tree_root() const
		{
			return !_parent && !_sibling;
		}
		bool is_forest_root() const
		{
			return !_parent && _sibling;
		}


	// Iterators and Ranges
	public:

		struct const_iterator_base
		{
		public:
			typedef const _Ty value_type;
			typedef value_type const & reference;
			typedef value_type const * pointer;
			typedef std::forward_iterator_tag iterator_category;
			typedef int difference_type;
		protected:
			basic_iterator(nullptr) {}
			basic_iterator(const pointer ptr) : pNode(ptr) {}
			pointer pNode;
		public:
			pointer get() const {
				return pNode;
			}

			bool is_valid() const
			{
				return pNode;
			}

			bool equal(const basic_iterator & rhs) const {
				return pNode == rhs.pNode;
			}
		};

		struct mutable_iterator_base
		{
		public:
			typedef _Ty value_type;
			typedef value_type& reference;
			typedef value_type* pointer;
			typedef int difference_type;
		protected:
			pointer pNode;
			basic_iterator(nullptr) {}
			basic_iterator(const pointer ptr) : pNode(ptr) {}
		public:
			pointer get() const {
				return pNode;
			}

			bool is_valid() const
			{
				return pNode;
			}

			bool equal (const mutable_iterator_base & rhs) const {
				return pNode == rhs.pNode;
			}
		};

		// _TBase must be basic_iterator or const_basic_iterator
		template <typename _TBase>
		class depth_first_iterator : public _TBase
		{
		public:
			typedef _TBase base_type;
			typedef depth_first_iterator self_type;
			typedef std::forward_iterator_tag iterator_category;
		public:
			depth_first_iterator(void) {}

			explicit depth_first_iterator(const pointer ptr)
				: base_type(ptr){}

			self_type operator ++(int) {
				self_type other(pNode);
				++(*this);
				return other;
			}

			self_type& operator ++(){
				move_to_next();
				return *this;
			}

			void move_to_next()
			{
				if (!pNode) return;
				if (pNode->_child)
				{
					pNode = pNode->_child;
				}
				else if (pNode->_sibling) pNode = pNode->_sibling;
				else
				{
					while ((pNode->_parent) && (!pNode->_parent->_sibling || pNode->_parent->_sibling == pNode))
						pNode = pNode->_parent;
					if (pNode->_parent)
						pNode = pNode->_parent->_sibling;
					else
						pNode = nullptr;
				}
			}

			void move_to_subtree_end()
			{
				auto pNode = pNode->_child;
				while ((pNode->_parent) && (!pNode->_parent->_brother || pNode->_parent->_brother == pNode))
					pNode = pNode->_parent;
				if (pNode->_parent)
					pNode = pNode->_parent->_brother;
				else
					pNode = nullptr;
			}

			reference operator * () const {
				return *pNode;
			}
			pointer operator -> () const {
				return pNode;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return pNode == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return pNode != rhs.get();
			}
		};

		template <typename _TBase>
		class breadth_first_iterator : public _TBase
		{
		public:
			typedef _TBase base_type;
			typedef std::forward_iterator_tag iterator_category;
			typedef breadth_first_iterator self_type;
		private:
			std::queue<pointer> node_queue;
			bool				ignore_sibling;
		public:
			width_first_iterator(void)
				: base_type(nullptr)
			{}

			// When ignore_root_sibling is set to True, the BFS-travel will ingore the siblings of pNode
			explicit width_first_iterator(const pointer ptr, bool ignore_root_sibling = false)
				: base_type(ptr), ignore_sibling(ignore_root_sibling)
			{
				node_queue.push(nullptr);
			}

			// Copy and bfs-iterator is expensive!
			width_first_iterator(const self_type& rhs)
				: base_type(ptr) , node_queue(rhs)
			{}

			width_first_iterator(self_type&& rhs)
				: base_type(ptr), node_queue(std::move(rhs))
			{}

			// Copy and bfs-iterator is expensive!
			self_type& operator=(const self_type& rhs)
			{
				node_queue = rhs;
				pNode = rhs.pNode;
			}
			self_type& operator=(self_type&& rhs)
			{
				node_queue = std::move(rhs);
				pNode = rhs.pNode;
			}

			// Copy and bfs-iterator is expensive!
			self_type operator ++(int) {
				self_type other(pNode);
				++(*this);
				return other;
			}

			self_type& operator ++() {
				move_to_next();
				return *this;
			}

			void move_to_next()
			{
				if (!pNode) return;
				if (ignore_sibling)
				{
					pNode = pNode->_child;
					ignore_sibling = false;
				}
				if (pNode->_sibling)
				{
					node_queue.push(pNode->_child);
					pNode = pNode->_sibling;
				}
				else if (!node_queue.empty())
				{
					pNode = node_queue.front();
					node_queue.pop();
				}
				pNode = nullptr;
			}

			reference operator * () const {
				return *pNode;
			}
			pointer operator -> () const {
				return pNode;
			}
			reference operator * () const {
				return *pNode;
			}
			pointer operator -> () const {
				return pNode;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return pNode == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return pNode != rhs.get();
			}
		};

		// Bidirectional-Sibling-Iterator 
		template <typename _TBase>
		class sibling_iterator : public _TBase
		{
		public:
			typedef _TBase base_type;
			typedef sibling_iterator self_type;
			typedef std::bidirectional_iterator_tag iterator_category;
		public:
			sibling_iterator(void)
				: base_type(nullptr)
			{}

			explicit sibling_iterator(const pointer ptr)
				: base_type(ptr)
			{}

			self_type operator ++(int) {
				self_type other(pNode);
				++(*this);
				return other;
			}

			self_type& operator ++() {
				if (!pNode) return;
				pNode = pNode->_sibling;
				return *this;
			}
			self_type operator --(int) {
				self_type other(pNode);
				--(*this);
				return other;
			}

			self_type& operator --() {
				if (!pNode) return;
				pNode = pNode->prev_sibling();
				return *this;
			}

			reference operator * () {
				return *pNode;
			}
			pointer operator -> () {
				return pNode;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return pNode == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return pNode != rhs.get();
			}
		};

		typedef depth_first_iterator<mutable_iterator_base>		mutable_depth_first_iterator;
		typedef breadth_first_iterator<mutable_iterator_base>	mutable_breadth_first_iterator;
		typedef sibling_iterator<mutable_iterator_base>			mutable_sibling_iterator;
		typedef depth_first_iterator<const_iterator_base>		const_depth_first_iterator;
		typedef breadth_first_iterator<const_iterator_base>		const_breadth_first_iterator;
		typedef sibling_iterator<const_iterator_base>			const_sibling_iterator;

		// Immutable ranges
		const_sibling_iterator children_begin() const{
			return const_sibling_iterator(this->_child);
		}

		const_sibling_iterator children_end() const{
			return const_sibling_iterator(nullptr);
		}

		const_depth_first_iterator descendants_begin() const {
			return const_depth_first_iterator(this->_child);
		}

		const_depth_first_iterator descendants_end() const {
			return const_depth_first_iterator(this);
		}
		// breadth_first_iterator can self determine if it has meet the end
		const_breadth_first_iterator descendants_breadth_first_begin() const {
			return const_breadth_first_iterator(this->_child);
		}
		// just an null-iterator
		const_sibling_iterator descendants_breath_first_end() const {
			return const_sibling_iterator(nullptr);
		}
		// Depth first begin iterator to all nodes inside this sub-tree
		const_depth_first_iterator begin() const {
			return const_depth_first_iterator(this);
		}
		// Depth first end iterator to all nodes inside this sub-tree
		const_depth_first_iterator end() const {
			auto itr = const_depth_first_iterator(this);
			itr.move_to_subtree_end();
			return itr;
		}
		// breadth_first_iterator can self determine if it has meet the end
		const_breadth_first_iterator breadth_first_begin() const {
			return const_breadth_first_iterator(this,true);
		}
		// just an null-iterator
		const_sibling_iterator breath_first_end() const {
			return const_sibling_iterator(nullptr);
		}

		// Mutable ranges
		// List-like iterator over children
		mutable_sibling_iterator children_begin() {
			return mutable_sibling_iterator(this->_child);
		}
		// List-like iterator over children
		mutable_sibling_iterator children_end() {
			return mutable_sibling_iterator(nullptr);
		}
		// Depth first descendants begin iterator
		mutable_depth_first_iterator descendants_begin() {
			return mutable_depth_first_iterator(this->_child);
		}
		// Depth first descendants end iterator
		mutable_depth_first_iterator descendants_end() {
			return mutable_depth_first_iterator(this);
		}
		// Breadth first descendants iterator can self determine if it has meet the end
		mutable_breadth_first_iterator descendants_breadth_first_begin() {
			return mutable_breadth_first_iterator(this->_child);
		}
		// just an null-iterator
		mutable_sibling_iterator descendants_breath_first_end() {
			return mutable_sibling_iterator(nullptr);
		}
		// begin iterator to all nodes inside this sub-tree
		mutable_depth_first_iterator begin() {
			return mutable_depth_first_iterator(this);
		}
		// end iterator to all nodes inside this sub-tree
		mutable_depth_first_iterator end() {
			auto itr = mutable_depth_first_iterator(this);
			itr.move_to_subtree_end();
			return itr;
		}
		// breadth_first_iterator can self determine if it has meet the end
		mutable_breadth_first_iterator breadth_first_begin() const {
			return const_breadth_first_iterator(this, true);
		}
		// just an null-iterator
		mutable_sibling_iterator breath_first_end() const {
			return mutable_sibling_iterator(nullptr);
		}


		// use this method to extract this node (and all of its sub node) as a sub-tree 
		// and remove it from the oringinal parent
		// this method ensure the rest of tree structure is not affected
		void isolate() {
			if (!this->_parent)
#ifdef _DEBUG
				throw new std::exception("Can not separate the root tree_node.");
#else
				return;
#endif

			if (this->_parent->_child == this) {
				this->_parent->_child = this->_sibling;
			}
			else
			{
				this->_parent->_sibling = this->_sibling;
			}
			if (this->_sibling)
				this->_sibling->_parent = this->_parent;
			this->_parent = nullptr;
			this->_sibling = nullptr;
		}

		inline void append_children_front(tree_node* pForest)
		{
			assert(pForest && !pForest->_parent);
			auto rptr = pForest;
			while (rptr->_sibling)
				rptr = rptr->_sibling;
			if (rptr)
				rptr->_sibling = this->_child;
			if (this->_child)
				this->_child->_parent = rptr;
			this->_child = pForest;
			pForest->_parent = this;
		}

		inline void append_children_back(tree_node* pForest)
		{
			assert(pForest && !pForest->_parent);
			tree_node* p = this->_child;
			if (p == nullptr) {
				this->_child = ptr;
				ptr->_parent = this;
			}
			else {
				while (p->_sibling != nullptr)
					p = p->_sibling;
				p->_sibling = ptr;
				ptr->_parent = p;
			}
		}

		inline void insert_as_siblings_after(tree_node* pForest)
		{
			assert(pForest && !pForest->_parent);
			pForest->_parent = this;
			if (_sibling)
			{
				auto rptr = pForest;
				while (rptr->_sibling)
					rptr = rptr->_sibling;
				rptr->_sibling = this->_sibling;
			}
			this->_sibling = pForest;
		}

		inline void insert_as_siblings_before(tree_node* pForest)
		{
			assert(pForest && !pForest->_parent);
			auto rptr = pForest;
			while (rptr->_sibling)
				rptr = rptr->_sibling;

			if (!this->_parent)
			{
				rptr->_sibling = this;
				this->_parent = pForest;
			} else {
				pForest->_parent = this->_parent;
				rptr->_sibling = this;
				if (this->_parent->_sibling == this)
					this->_parent->sibling = pTree;
				else // this->parent->child == this
					this->_parent->_child = pTree;
				this->_parent = rptr;
			}
		}
	};

}