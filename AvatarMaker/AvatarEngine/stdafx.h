// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#define NOMINMAX						// Exclude the silly max , min macro
// Windows Header Files:
#include <windows.h>
//#include <ole2.h>
//#include <Shlobj.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

// C++ RunTime Header
#include <memory>
#include <exception>
#include <iterator>
#include <functional>
#include <type_traits>
#include <algorithm>
#include <iostream>
#include <iomanip>

// STL Containers
#include <vector>
#include <deque>
#include <list>
#include <array>
#include <map>
#include <stack>
#include <queue>
#include <string>
#include <sstream>

// Concurrency Runtime
#include <thread>
#include <mutex>
#include <ppl.h>
#include <ppltasks.h>

// DirectX Header Files
#include <d3d11_1.h>

// DirectX Math Libary
#include <DirectXMath.h>
#include <DirectXColors.h>
#include <DirectXCollision.h>


// Additional math libary
#include <Eigen\Sparse> // Sparse matrix 
#include <boost\graph\adjacency_list.hpp> // Boost Graph Libary