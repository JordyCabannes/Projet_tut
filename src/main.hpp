#ifndef MAIN_HPP
#define MAIN_HPP

#include <cstdlib>
#include <cstdio>

#include "graph_wrapper.h"
#include "reglc_graph.h"
#include "GraphFactory.h"

int main(int argc, char ** argv);

void solve(const Transport::Graph * dbGraph);

#endif/*MAIN_HPP*/
