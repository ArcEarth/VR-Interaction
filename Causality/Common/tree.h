#pragma once
#include <iterator>
#include <vector>
#include <queue>
#include <stack>
#include <cassert>

namespace stree
{
	struct tree_node_relation_descants_ownership {};
	struct tree_node_relation_no_decants_ownership {};
	// Use and only it as the base of your tree node like:
	// class Ty : public tree_node<Ty>

	template <class _TItr>
	class iterator_range
	{
	public:
		typedef _TItr iterator_type;
		typedef _TItr const_iterator;
		typedef typename _TItr::value_type value_type;
	protected:
		_TItr _begin;
		_TItr _end;
	public:
		iterator_range(_TItr&& begin, _TItr&& end)
			: _begin(std::move(begin)), _end(std::move(end))
		{}

		iterator_range(iterator_range&& rhs)
		{
			_begin = std::move(rhs._begin);
			_end = std::move(rhs._end);
		}

		_TItr begin() { return std::move(_begin); }
		_TItr end() { return std::move(_end); }

	};


	template<typename _Ty, bool _DescendabtsOwnership = true>
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
		pointer _sibling;
		// First child node
		pointer _child;
		// Physical parent node of this node in data structure
		// May be Logical Parent or Prev Sibling
		pointer _parent;
		//		tree_node *_next;

	private:
		template <typename T>
		inline static void delete_s(std::enable_if_t<_DescendabtsOwnership, T>* &pData) {
			if (pData) {
				delete pData;
				pData = nullptr;
			}
		}
		template <typename T>
		inline static void delete_s(std::enable_if_t<!_DescendabtsOwnership, T>* &pData) {
			pData = nullptr;
		}

	// Basic Properties
	public:

		tree_node()
			: _parent(nullptr), _sibling(nullptr), _child(nullptr)
		{
		}

		~tree_node()
		{
			delete_s<_Ty>(_sibling);
			delete_s<_Ty>(_child);
#ifdef _DEBUG
			_parent = nullptr;
#endif
		}

		tree_node(const tree_node& rhs)
			: Entity(rhs.Entity), _parent(nullptr), _sibling(nullptr), _child(nullptr)
		{
			rhs.for_all_children([&](const tree_node& SrcNode)
			{
				tree_node *current = new tree_node(SrcNode);
				this->_insert_as_child_back(current);
			});
		}

		tree_node& operator = (const tree_node& rhs)
		{
			this->Entity = rhs.Entity;
			this->_parent = nullptr;
			this->_sibling = nullptr;
			rhs.for_all_children([&](tree_node& SrcNode)
			{
				tree_node *current = new tree_node(SrcNode);
				this->children_append_front(current);
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
		bool has_child() const { return _child; }
		bool is_leaf() const { return !_child; }
		// if this node is Logical Root
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
			const_iterator_base() {}
			const_iterator_base(const pointer ptr) : current(ptr) {}
			pointer current;
		public:
			pointer get() const {
				return current;
			}

			bool is_valid() const
			{
				return current;
			}

			template <class _TItr>
			bool equal(const _TItr & rhs) const {
				return current == rhs.current;
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
			pointer current;
			mutable_iterator_base() {}
			mutable_iterator_base(const pointer ptr) : current(ptr) {}
		public:
			pointer get() const {
				return current;
			}

			bool is_valid() const
			{
				return current;
			}

			template <class _TItr>
			bool equal(const _TItr & rhs) const {
				return current == rhs.current;
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
				self_type other(current);
				++(*this);
				return other;
			}

			self_type& operator ++(){
				move_to_next();
				return *this;
			}

			void move_to_next()
			{
				if (!current) return;
				if (current->_child)
				{
					current = current->_child;
				}
				else if (current->_sibling) current = current->_sibling;
				else
				{
					// move_to_next_from_this_subtree
					while ((current->_parent) && (!current->_parent->_sibling || current->_parent->_sibling == current))
						current = current->_parent;
					if (current->_parent)
						current = current->_parent->_sibling;
					else
						current = nullptr;
				}
			}

			inline void move_to_next_from_this_subtree()
			{
				if (current->_sibling)
					current = current->_sibling;
				else {
					while ((current->_parent) && (!current->_parent->_sibling || current->_parent->_sibling == current))
						current = current->_parent;
					if (current->_parent)
						current = current->_parent->_sibling;
					else
						current = nullptr;
				}
			}

			reference operator * () const {
				return *current;
			}
			pointer operator -> () const {
				return current;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return current == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return current != rhs.get();
			}
		};
		template <typename _TBase>
		class leaf_iterator : public _TBase
		{
		public:
			typedef _TBase base_type;
			typedef leaf_iterator self_type;
			typedef std::forward_iterator_tag iterator_category;
		public:
			leaf_iterator(void) {}

			explicit leaf_iterator(const pointer ptr)
				: base_type(ptr) {
				while (current && current->_child)
					current = current->_child;
			}

			self_type operator ++(int) {
				self_type other(current);
				++(*this);
				return other;
			}

			self_type& operator ++() {
				move_to_next();
				return *this;
			}

			void move_to_next()
			{
				if (!current) return;

				if (current->_child)
				{
					current = current->_child;
					while(current->_child)
						current = current->_child;
				}
				else 
					move_to_next_from_this_subtree();
			}

			// when all the desendants of this node is visited
			inline void move_to_next_from_this_subtree()
			{
				if (current->_sibling)
				{
					current = current->_sibling;
					// move to next
					//if (current->_child)
					//	move_to_next();
					while (current->_child)
						current = current->_child;
				}
				else
				{
					while ((current->_parent) && (!current->_parent->_sibling || current->_parent->_sibling == current))
						current = current->_parent;
					if (current->_parent) //  && current->_parent->_sibling && current->_parent->_sibling != current
					{
						current = current->_parent->_sibling;
						// move to next
						//if (current && current->_child)
						//	move_to_next();
						while (current->_child)
							current = current->_child;

					}
					else
						current = nullptr;
				}
			}

			reference operator * () const {
				return *current;
			}
			pointer operator -> () const {
				return current;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return current == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return current != rhs.get();
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
			breadth_first_iterator(void)
				: base_type(nullptr)
			{}

			// When ignore_root_sibling is set to True, the BFS-travel will ingore the siblings of current
			explicit breadth_first_iterator(const pointer ptr, bool ignore_root_sibling = false)
				: base_type(ptr), ignore_sibling(ignore_root_sibling)
			{
				node_queue.push(nullptr);
			}

			// Copy and bfs-iterator is expensive!
			breadth_first_iterator(const self_type& rhs)
				: base_type(ptr) , node_queue(rhs)
			{}

			breadth_first_iterator(self_type&& rhs)
				: base_type(ptr), node_queue(std::move(rhs))
			{}

			// Copy and bfs-iterator is expensive!
			self_type& operator=(const self_type& rhs)
			{
				node_queue = rhs;
				current = rhs.current;
			}
			self_type& operator=(self_type&& rhs)
			{
				node_queue = std::move(rhs);
				current = rhs.current;
			}

			// Copy and bfs-iterator is expensive!
			self_type operator ++(int) {
				self_type other(current);
				++(*this);
				return other;
			}
			self_type& operator ++() {
				move_to_next();
				return *this;
			}

			void move_to_next()
			{
				if (!current) return;
				if (ignore_sibling)
				{
					current = current->_child;
					ignore_sibling = false;
				}
				if (current->_sibling)
				{
					node_queue.push(current->_child);
					current = current->_sibling;
				}
				else if (!node_queue.empty())
				{
					current = node_queue.front();
					node_queue.pop();
				}
				current = nullptr;
			}

			reference operator * () const {
				return *current;
			}
			pointer operator -> () const {
				return current;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return current == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return current != rhs.get();
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
				self_type other(current);
				++(*this);
				return other;
			}

			self_type& operator ++() {
				if (!current) return *this;
				current = current->_sibling;
				return *this;
			}
			self_type operator --(int) {
				self_type other(current);
				--(*this);
				return other;
			}

			self_type& operator --() {
				if (!current) return;
				current = current->prev_sibling();
				return *this;
			}

			reference operator * () const  {
				return *current;
			}
			pointer operator -> () const  {
				return current;
			}

			template <class _TItr>
			bool operator == (const _TItr & rhs) const {
				return current == rhs.get();
			}

			template <class _TItr>
			bool operator != (const _TItr & rhs) const {
				return current != rhs.get();
			}
		};

		typedef depth_first_iterator<mutable_iterator_base>		mutable_depth_first_iterator;
		typedef breadth_first_iterator<mutable_iterator_base>	mutable_breadth_first_iterator;
		typedef sibling_iterator<mutable_iterator_base>			mutable_sibling_iterator;
		typedef leaf_iterator<mutable_iterator_base>			mutable_leaf_iterator;
		typedef depth_first_iterator<const_iterator_base>		const_depth_first_iterator;
		typedef breadth_first_iterator<const_iterator_base>		const_breadth_first_iterator;
		typedef sibling_iterator<const_iterator_base>			const_sibling_iterator;
		typedef leaf_iterator<const_iterator_base>				const_leaf_iterator;

		typedef const_depth_first_iterator const_iterator;
		typedef mutable_depth_first_iterator iterator;

		// Immutable ranges

		// iterator throught this and all it's "next" siblings 
		const_sibling_iterator siblings_begin() const {
			return const_sibling_iterator(static_cast<const_pointer>(this));
		}

		const_sibling_iterator siblings_end() const {
			return const_sibling_iterator(nullptr);
		}

		// Iterator though all it's direct children
		const_sibling_iterator children_begin() const{
			return const_sibling_iterator(static_cast<const_pointer>(this)->_child);
		}

		const_sibling_iterator children_end() const{
			return const_sibling_iterator(nullptr);
		}
		const_leaf_iterator leaves_begin() const
		{
			return const_leaf_iterator(static_cast<const_pointer>(this));
		}
		const_leaf_iterator leaves_end() const
		{
			auto itr = const_leaf_iterator(static_cast<const_pointer>(this));
			itr.move_to_next_from_this_subtree();
			return itr;
		}

		const_depth_first_iterator descendants_begin() const {
			return const_depth_first_iterator(static_cast<const_pointer>(this)->_child);
		}

		const_depth_first_iterator descendants_end() const {
			return const_depth_first_iterator(static_cast<const_pointer>(this));
		}
		// breadth_first_iterator can self determine if it has meet the end
		const_breadth_first_iterator descendants_breadth_first_begin() const {
			return const_breadth_first_iterator(static_cast<const_pointer>(this)->_child);
		}
		// just an null-iterator
		const_sibling_iterator descendants_breath_first_end() const {
			return const_sibling_iterator(nullptr);
		}
		// Depth first begin iterator to all nodes inside this sub-tree
		const_depth_first_iterator begin() const {
			return const_depth_first_iterator(static_cast<const_pointer>(this));
		}
		// Depth first end iterator to all nodes inside this sub-tree
		const_depth_first_iterator end() const {
			auto itr = const_depth_first_iterator(static_cast<const_pointer>(this));
			itr.move_to_next_from_this_subtree();
			return itr;
		}
		// breadth_first_iterator can self determine if it has meet the end
		const_breadth_first_iterator breadth_first_begin() const {
			return const_breadth_first_iterator(static_cast<const_pointer>(this),true);
		}
		// just an null-iterator
		const_sibling_iterator breath_first_end() const {
			return const_sibling_iterator(nullptr);
		}

		// Mutable ranges

		// iterator throught this and all it's "next" siblings 
		mutable_sibling_iterator siblings_begin()  {
			return mutable_sibling_iterator(static_cast<pointer>(this));
		}

		mutable_sibling_iterator siblings_end()  {
			return mutable_sibling_iterator(nullptr);
		}
		// List-like iterator over children
		mutable_sibling_iterator children_begin() {
			return mutable_sibling_iterator(static_cast<pointer>(this)->_child);
		}
		// List-like iterator over children
		mutable_sibling_iterator children_end() {
			return mutable_sibling_iterator(nullptr);
		}
		mutable_leaf_iterator leaves_begin() 
		{
			return mutable_leaf_iterator(static_cast<pointer>(this));
		}
		mutable_leaf_iterator leaves_end() 
		{
			auto itr = mutable_leaf_iterator(static_cast<pointer>(this));
			itr.move_to_next_from_this_subtree();
			return itr;
		}
		// Depth first descendants begin iterator
		mutable_depth_first_iterator descendants_begin() {
			return mutable_depth_first_iterator(static_cast<pointer>(this)->_child);
		}
		// Depth first descendants end iterator
		mutable_depth_first_iterator descendants_end() {
			return mutable_depth_first_iterator(static_cast<pointer>(this));
		}
		// Breadth first descendants iterator can self determine if it has meet the end
		mutable_breadth_first_iterator descendants_breadth_first_begin() {
			return mutable_breadth_first_iterator(static_cast<pointer>(this)->_child);
		}
		// just an null-iterator
		mutable_sibling_iterator descendants_breath_first_end() {
			return mutable_sibling_iterator(nullptr);
		}
		// begin iterator to all nodes inside this sub-tree
		mutable_depth_first_iterator begin() {
			return mutable_depth_first_iterator(static_cast<pointer>(this));
		}
		// end iterator to all nodes inside this sub-tree
		mutable_depth_first_iterator end() {
			auto itr = mutable_depth_first_iterator(static_cast<pointer>(this));
			itr.move_to_next_from_this_subtree();
			return itr;
		}
		// breadth_first_iterator can self determine if it has meet the end
		mutable_breadth_first_iterator breadth_first_begin()  {
			return const_breadth_first_iterator(static_cast<pointer>(this), true);
		}
		// just an null-iterator
		mutable_sibling_iterator breath_first_end()  {
			return mutable_sibling_iterator(nullptr);
		}

		iterator_range<const_sibling_iterator>
			children() const
		{
			return iterator_range<const_sibling_iterator>(children_begin(), children_end());
		}
		iterator_range<const_depth_first_iterator>
			nodes_in_tree() const
		{
			return iterator_range<const_depth_first_iterator>(begin(), end());
		}
		iterator_range<const_leaf_iterator>
			leaves() const
		{
			return iterator_range<const_leaf_iterator>(leaves_begin(), leaves_end());
		}
		iterator_range<const_depth_first_iterator>
			descendants() const
		{
			return iterator_range<const_depth_first_iterator>(descendants_begin(), descendants_end());
		}
		iterator_range<mutable_sibling_iterator>
			children()
		{
			return iterator_range<mutable_sibling_iterator>(children_begin(), children_end());
		}
		iterator_range<mutable_depth_first_iterator>
			nodes_in_tree()
		{
			return iterator_range<mutable_depth_first_iterator>(begin(), end());
		}
		iterator_range<mutable_leaf_iterator>
			leaves()
		{
			return iterator_range<mutable_leaf_iterator>(leaves_begin(), leaves_end());
		}
		iterator_range<mutable_depth_first_iterator>
			descendants()
		{
			return iterator_range<mutable_depth_first_iterator>(descendants_begin(), descendants_end());
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