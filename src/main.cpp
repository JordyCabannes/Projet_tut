#include "main.hpp"

int main(int argc, char ** argv)
{
  if(argc == 3)
    {
      std::string graph_storage_type = argv[2];
      
      bool isTxtStorage = false;
      bool isBinStorage = false;
      
      if(graph_storage_type == "txt")
	{
	  isTxtStorage = true;
	}
      else if(graph_storage_type == "bin")
	{
	  isBinStorage = true;
	}
      else
	{
	  std::cout << "Undefined input graph type." << std::endl;
	  
	  return EXIT_FAILURE;
	}
      
      Transport::GraphFactory graphFactory(argv[1], isBinStorage);
      const Transport::Graph * graph = graphFactory.get();
      
      std::cout << "Graph loaded";
      
      solve(graph);
      
      delete graph;
    }
  else
    {
      std::cout << "Wrong number of arguments (2 expected)." << std::endl;
      
      return EXIT_FAILURE;
    }
  
  std::cout << std::flush;
  
  return EXIT_SUCCESS;
}

void solve(const Transport::Graph * dbGraph)
{
  std::cout << "Solving" << std::endl;
  
  Graph_t::out_edge_iterator it;
  Graph_t::out_edge_iterator end;
  
  tie(it, end) = boost::out_edges(4, dbGraph->g);
  
  while(it != end)
    {
      std::cout << *it;
      ++it;
    }

  std::cout << std::endl;
}
