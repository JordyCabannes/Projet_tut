#include "RegLC/AlgoTypedefs.h"
#include "DataStructures/GraphFactory.h"
#include "MultipleParticipants/run_configurations.h"
#include "RegLC/reglc_graph.h"
#include "RegLC/DRegLC.h"
#include "MultipleParticipants/muparo.h"
#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/concept_check.hpp>
#include <boost/lexical_cast.hpp>

#include "MuparoTypedefs.h"
#include "node_filter_utils.h"
#include <AspectTargetAreaLandmark.h>
#include "AspectTargetAreaStop.h"
#include "../MultiObjectives/Martins.h"

using RLC::DRegLC;
using RLC::AspectCount;
using RLC::LandmarkSet;



/*----------------------------------------------------------------------------
	Constantes utilisées au cour de l'exécution de l'algorithme
-----------------------------------------------------------------------------*/

//L'ensemble de ces constantes sont utilisées pour le covoiturage avec 3 noeuds de pick-up et un noeud de de drop-off

//Permet de récupérer le coût entre l'origine voiture et le meilleur point pour le premier ramassage
int cost_pick_up1;

//Permet de récupérer le coût entre l'origine voiture et le meilleur point pour le deuxième ramassage
int cost_pick_up2;

//Permet de récupérer le coût entre l'origine du premier piéton récupéré et le meilleur noeud pour le premier ramassage
int cost_ped_pick_up1;

//Permet de récupérer le coût entre l'origine second piéton récupéré et le meilleur point pour le second ramassage
int cost_ped_pick_up2;

//Permet de récupérér le meilleur noeud pour le premier ramassage
int pick_up1;

//Permet de récupérer le meilleur noeud pour le second ramassage
int pick_up2;

//Permet de savoir à quel piéton appartient le premier point de pick-up
int node_pick_up1;

//Permet de savoir à quel piéton appartient le second point de pick-up
int node_pick_up2;


//Permet de récupérer l'origine du piéton qui n'a pas encore été récupéré
int ped_rest;

//Permet de définir le nombre d'instances de tests qui seront utilisées 
const int nb_instances=10;


/*-----------------------------------------------------------------------------
	Structures pour la phase de tests
------------------------------------------------------------------------------*/

//structure de tests pour l'algorithme initialement présent sur la plateforme
typedef struct
{
        int src_car;
        int dest_car;
        int src_ped;
        int dest_ped;
	int scenario_id;
	int car_od_cost;
	int ped_od_cost;

} structure_test;


//structure de tests pour les algorithmes : - 3 piétons 1 point de pick-up, 1 point de drop-off
//					    - 3 piétons 3 points de pick-up, 1 point de drop-off
typedef struct
{
	int scenario_id;
        int src_car;
        int dest_car;
        int src_ped_1;;
        int dest_ped_1;
	int src_ped_2;
        int dest_ped_2;
	int src_ped_3;
        int dest_ped_3;
        int car_od_cost;
        int ped_1_od_cost;
	int ped_2_od_cost;
	int ped_3_od_cost;

} structure_test_a1;

//structure pour contenir le résultat de l'algorithme initialement présent sur la plateforme
typedef struct
{
        int scenario_id;
        int pick_up;
        int drop_off;
        int cost_tot;
        int cost_a1;
        int cost_a2;
        int cost_a3;
	int cost_a4;
	int cost_a5;
	int tps_exec;

} structure_result_algo;


//structure pour contenir les résultats l'algorithme : - 3 piétons 1 point de pick-up, 1 point de drop-off
typedef struct
{
        int scenario_id;
        int pick_up;
        int drop_off;
        int cost_tot;
        int cost_a1;
        int cost_a2;
        int cost_a3;
        int cost_a4;
        int cost_a5;
	int cost_a6;
	int cost_a7;
        int tps_exec;

} structure_result_algo_v1;


//structure pour contenir les résultats l'algorithme : - 3 piétons 3 points de pick-up, 1 point de drop-off
typedef struct
{
        int scenario_id;
        int pick_up_1;
	int pick_up_2;
	int pick_up_3;
        int drop_off;
        int cost_tot;
        int cost_a1;
        int cost_a2;
        int cost_a3;
        int cost_a4;
        int cost_a5;
        int cost_a6;
        int cost_a7;
	int cost_a8;
	int cost_a9;
        int tps_exec;

} structure_result_algo_v2;


//structure qui va contenir les résultats de la fonction covoit_gen
typedef struct
{
	//contient le coût entre l'origine voiture et le pick-up qui suit
	int cost;
	//contient le noeud de pick-up
	int pick_up;
	//contient le coût entre l'origine piéton et le pick-up précédent
	int cost_ped;

} retour;

/*-------------------------------------------------------------------------------------
	Prototypes des fonctions de covoiturage
--------------------------------------------------------------------------------------*/

// fonction pour le covoiturage avec avec 3 piétons, 1 seul point de pick-up et un seul point de drop-off
void covoiturage_3ped_1dri_spick_sdrop(string filename, structure_test_a1 test, const Transport::Graph * gr);

// fonction pour le covoiturage avec avec 1 piéton, 1 seul point de pick-up et un seul point de drop-off
void covoiturage_simple(string filename, structure_test test, const Transport::Graph * gr);

// fonction pour le covoiturage avec avec 3 piétons, 3 points de pick-up et un seul point de drop-off
void covoiturage_3ped_1dri_difpick_sdrop(string filename, structure_test_a1 test, const Transport::Graph * gr);

//fonction qui renvoi une struct retour à l'issu d'un covoiturage simple
retour covoit_gen(int src_ped, int src_car,int dest_ped, int dest_car, const Transport::Graph * gr);


/*--------------------------------------------------------------------------------------
	Fonctions pour initialiser les covoiturages 
---------------------------------------------------------------------------------------*/

//fonction qui initialise le covoiturage avec 3 piétons, 1 point de pick-up et 1 point de drop-off
template<typename T>
void init_car_sharing_3ped_1driv_sdrop_spick(T * cs, const Transport::Graph* trans, structure_test_a1 test_a1, RLC::DFA dfa_ped, RLC::DFA dfa_car )
{
    cs->vres.a_nodes.push_back(test_a1.src_ped_1);
    cs->vres.a_nodes.push_back(test_a1.src_ped_2);
    cs->vres.a_nodes.push_back(test_a1.src_ped_3);
    cs->vres.a_nodes.push_back(test_a1.src_car);
    cs->vres.b_nodes.push_back(test_a1.dest_ped_1);
    cs->vres.b_nodes.push_back(test_a1.dest_ped_2);
    cs->vres.b_nodes.push_back(test_a1.dest_ped_3);
    cs->vres.b_nodes.push_back(test_a1.dest_car);

    int day = 10;
    //int time = 50000;
    int _time = 0;

    //Création de 1 graphe pour chaque origine piéton avec g1, g2, g3
    RLC::Graph *g1 = new RLC::Graph(cs->transport, dfa_ped );
    RLC::Graph *g2 = new RLC::Graph(cs->transport, dfa_ped );
    RLC::Graph *g3 = new RLC::Graph(cs->transport, dfa_ped );
    RLC::Graph *g4 = new RLC::Graph(cs->transport, dfa_car );
    RLC::Graph *g5 = new RLC::Graph(cs->transport, dfa_car );
    RLC::BackwardGraph *g6 = new RLC::BackwardGraph(g4);

    //Création d'un graphe pour chaque destination piéton avec g7, g8, g9
    RLC::Graph *g7 = new RLC::Graph(cs->transport, dfa_ped );
    RLC::Graph *g8 = new RLC::Graph(cs->transport, dfa_ped );
    RLC::Graph *g9 = new RLC::Graph(cs->transport, dfa_ped );

    cs->graphs.push_back( g1 );
    cs->graphs.push_back( g2 );
    cs->graphs.push_back( g3 );
    cs->graphs.push_back( g4 );
    cs->graphs.push_back( g5 );
    cs->graphs.push_back( g6 );
    cs->graphs.push_back( g7 );
    cs->graphs.push_back( g8 );
    cs->graphs.push_back( g9 );


    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g1, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g2, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g3, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g4, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g5, day, 2)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g6, day, 1)) ) );
//  cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g5, day, 1)) ) );
    //On exécutera un algorithme de Martins entre le point de drop-off et chaque noeud destination d'un piéton
    cs->dij.push_back( new RLC::Martins(g7, test_a1.dest_ped_1, day) );
    cs->dij.push_back( new RLC::Martins(g8, test_a1.dest_ped_2, day) );
    cs->dij.push_back( new RLC::Martins(g9, test_a1.dest_ped_3, day) );


    cs->insert( StateFreeNode(0, test_a1.src_ped_1), _time, 0);
    cs->insert( StateFreeNode(1, test_a1.src_car), _time, 0);
    cs->insert( StateFreeNode(2, test_a1.src_ped_2), _time, 0);
    cs->insert( StateFreeNode(3, test_a1.src_ped_3), _time, 0);
    cs->insert( StateFreeNode(4, test_a1.dest_car), 0, 0);
}


//fonction qui initialise le covoiturage avec 3 piétons, 3 points de pick-up et 1 point de drop-off
template<typename T>
void init_car_sharing_3ped_1driv_sdrop_difpick(T * cs, const Transport::Graph* trans, structure_test_a1 test_a1, RLC::DFA dfa_ped, RLC::DFA dfa_car )
{
    	
    //On récupère le point de pick-up du covoiturage entre le 1er piéon et la voiture, ainsi que le coût
    //origine piéton1 -> pick-up et origine voiture -> pick-up
    retour car_ped1=covoit_gen(test_a1.src_ped_1, test_a1.src_car, test_a1.dest_ped_1, test_a1.dest_car, trans);

    //Idem que précedemment mais cette fois covoiturage entre le piéton 2 et la voiture
    retour car_ped2=covoit_gen(test_a1.src_ped_2, test_a1.src_car, test_a1.dest_ped_2, test_a1.dest_car, trans);
    
    //Résultats du covoiturage entre le piéton 3 et la voiture
    retour car_ped3=covoit_gen(test_a1.src_ped_3, test_a1.src_car, test_a1.dest_ped_3, test_a1.dest_car, trans);	

    //on initialise cette variable qui va stocker le plus petit coût entre le piéton ramassé en premier et son
    //point de pick-up correspondant
    int min_1=car_ped1.cost;
    
    //Permettra de stocker le numéro du noeud choisi comme second point de pick-up
    int pick_up_2;

    //Permetta de sauvegarder le noeud de destination du dernier piéton qui sera récupéré
    int dest_ped_rest;
 
    //Permettra de sauvegarder le coût entre le premier point de pick-up choisi et le second point de pick-up sélectionné
    int min_2; 

    //Permet de récupérer le noeud du premier point de pick-up sélectionné
    int pick_up_1=car_ped1.pick_up;

    //On affecte pour l'instantà la constante le coût entre l'origine du piéton et son point de pick-up
    cost_ped_pick_up1=car_ped1.cost_ped;
    
    //Si l'origine de la voiture est plus proche du point de pick-up du piéton 2
    if(car_ped2.cost<min_1)
    {
      	min_1 =car_ped2.cost;
      	pick_up_1=car_ped2.pick_up;
    	cost_ped_pick_up1=car_ped2.cost_ped;
    }
    //Si l'origine de la voiture est plus proche du point de pick-up du piéton 3
    if(car_ped3.cost<min_1)
    {
      min_1 =car_ped3.cost;
      pick_up_1=car_ped3.pick_up;
      cost_ped_pick_up1=car_ped3.cost_ped;
    }

    //A ce stade nous avons sélectionné le premier point de pick-up du covoiturage final

    //Si le premier point de pick-up était celui du piéton 1 
    if(min_1==car_ped1.cost)
    {
	//On relance a fonction covoit-gen mais cette-fois ont prend pour origine voiture le premier point de pick-up
	//et ici comme origine piéton celle du piéton
	retour car_ped2_bis=covoit_gen(test_a1.src_ped_2, pick_up_1, test_a1.dest_ped_2, test_a1.dest_car, trans);
    	
	//Pareil que ci-dessus sauf que l'origine du piéton est celle du piéton 3
	retour car_ped3_bis=covoit_gen(test_a1.src_ped_3, pick_up_1, test_a1.dest_ped_3, test_a1.dest_car, trans);  
 	
	//On initialise la constante avec le point de pick-up du piéton 1 car on considère dans ce if que c'est
	//le meilleur premier point de ramassage
	node_pick_up1=test_a1.src_ped_1;
	
	//On suit le même mode opératoire que pour la sélection du premier point de pick-up	
        min_2=car_ped2_bis.cost;
    	pick_up_2=car_ped2_bis.pick_up;

	//Le piéton restant est pour l'instant le piéton 3
	ped_rest=test_a1.src_ped_3;
	
	node_pick_up2=test_a1.src_ped_2;	
	cost_ped_pick_up2=car_ped2_bis.cost_ped;

   	//Si le point de pick-up 3 est plus proche du premier point de pick-up
	if(car_ped3_bis.cost<min_2)
    	{
       	    min_2 =car_ped3_bis.cost;
            pick_up_2=car_ped3_bis.pick_up;
            ped_rest=test_a1.src_ped_2;
      	    node_pick_up2=test_a1.src_ped_3;
	    cost_ped_pick_up2=car_ped3_bis.cost_ped;		    
	}
    }
    //Si le premier point de pick-up était celui du piéton 2
    //On suit le même mode opératoire que précedemment
    else if (min_1==car_ped2.cost)
    {
    	retour car_ped1_bis=covoit_gen(test_a1.src_ped_1, pick_up_1, test_a1.dest_ped_1, test_a1.dest_car, trans);
        retour car_ped3_bis=covoit_gen(test_a1.src_ped_3, pick_up_1, test_a1.dest_ped_3, test_a1.dest_car, trans);
  	
	node_pick_up1=test_a1.src_ped_2;
        min_2=car_ped1_bis.cost;
        pick_up_2=car_ped1_bis.pick_up;
	
	node_pick_up2=test_a1.src_ped_1;
	cost_ped_pick_up2=car_ped1_bis.cost_ped;
        
	ped_rest=test_a1.src_ped_3;
	
	if(car_ped3_bis.cost<min_2)
        {
	    min_2 =car_ped3_bis.cost;
            pick_up_2=car_ped3_bis.pick_up;
            ped_rest=test_a1.src_ped_1;
	    node_pick_up2=test_a1.src_ped_3;
	    cost_ped_pick_up2=car_ped3_bis.cost_ped;
        }
   }
    //Si le premier point de pick-up était celui du piéton 1 
    else if (min_1==car_ped3.cost)
    {
        retour car_ped1_bis=covoit_gen(test_a1.src_ped_1, pick_up_1, test_a1.dest_ped_1, test_a1.dest_car, trans);
        retour car_ped2_bis=covoit_gen(test_a1.src_ped_2, pick_up_1, test_a1.dest_ped_2, test_a1.dest_car, trans);
        
	node_pick_up1=test_a1.src_ped_3;
	min_2=car_ped1_bis.cost;
        pick_up_2=car_ped1_bis.pick_up;
        
	ped_rest=test_a1.src_ped_2;
	cost_ped_pick_up2=car_ped1_bis.cost_ped;
        node_pick_up2=test_a1.src_ped_1;
	
	if(car_ped2_bis.cost<min_2)
        {
            min_2 =car_ped2_bis.cost;
            pick_up_2=car_ped2_bis.pick_up;
	    ped_rest=test_a1.src_ped_1;
	    node_pick_up2=test_a1.src_ped_2;
	    cost_ped_pick_up2=car_ped2_bis.cost_ped;
        }
    }

    //A ce stade nous avons sélectionné le meilleur premier point de pick-up et le meilleur second point de pick-up

    //On va maintenant récupérer le noeud de destination du piéton qui n'a pas encore été récupéré
    if(ped_rest==test_a1.src_ped_1)
    {
 	dest_ped_rest=test_a1.dest_ped_1;
    }
    else if(ped_rest==test_a1.src_ped_2)
    {
	dest_ped_rest=test_a1.dest_ped_2;
    } 
    else if(ped_rest==test_a1.src_ped_1)
    {
	dest_ped_rest=test_a1.dest_ped_3;
    }

    //On initialise les constantes déclarées plus haut
    cost_pick_up1=min_1;
    cost_pick_up2=min_2;
    pick_up1=pick_up_1;
    pick_up2=pick_up_2;

    //On initialise maintenant les paramètres nécessaires pour un covoiturage simple 
    //l'origine de la voiture sera le second point de pick-up et l'origine du piéton sera celle du piéton restant
    //la destination du piéton sera celle du piéton restant
    cs->vres.a_nodes.push_back(ped_rest);
    cs->vres.a_nodes.push_back(pick_up_2);
    cs->vres.b_nodes.push_back(dest_ped_rest);
    //cs->vres.b_nodes.push_back(test_a1.dest_ped_2);
    //cs->vres.b_nodes.push_back(test_a1.dest_ped_3);
    cs->vres.b_nodes.push_back(test_a1.dest_car);

    int day = 10;
    //int time = 50000;
    int _time = 0;

    RLC::Graph *g1 = new RLC::Graph(cs->transport, dfa_ped );
    //RLC::Graph *g2 = new RLC::Graph(cs->transport, dfa_ped );
    //RLC::Graph *g3 = new RLC::Graph(cs->transport, dfa_ped );
    RLC::Graph *g2 = new RLC::Graph(cs->transport, dfa_car );
    RLC::Graph *g3 = new RLC::Graph(cs->transport, dfa_car );
    RLC::BackwardGraph *g4 = new RLC::BackwardGraph(g2);
    RLC::Graph *g5 = new RLC::Graph(cs->transport, dfa_ped );
    //RLC::Graph *g6 = new RLC::Graph(cs->transport, dfa_ped );
    //RLC::Graph *g7 = new RLC::Graph(cs->transport, dfa_ped );

    cs->graphs.push_back( g1 );
    cs->graphs.push_back( g2 );
    cs->graphs.push_back( g3 );
    cs->graphs.push_back( g4 );
    cs->graphs.push_back( g5 );
    //cs->graphs.push_back( g6 );
    //cs->graphs.push_back( g7 );
    //cs->graphs.push_back( g8 );
    //cs->graphs.push_back( g9 );


    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g1, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g2, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g3, day, 1)) ) );
    cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g4, day, 1)) ) );
    //cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g5, day, 2)) ) );
    //cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g6, day, 1)) ) );
//  cs->dij.push_back( new typename T::Dijkstra( typename T::Dijkstra::ParamType(RLC::DRegLCParams(g5, day, 1)) ) );
    cs->dij.push_back( new RLC::Martins(g5, dest_ped_rest, day) );
    //cs->dij.push_back( new RLC::Martins(g6, test_a1.dest_ped_2, day) );
    //cs->dij.push_back( new RLC::Martins(g7, test_a1.dest_ped_3, day) );


    cs->insert( StateFreeNode(0, ped_rest), _time, 0);
    cs->insert( StateFreeNode(1, pick_up_2), _time, 0);
   // cs->insert( StateFreeNode(2, test_a1.src_ped_2), _time, 0);
    //cs->insert( StateFreeNode(3, test_a1.src_ped_3), _time, 0);
    cs->insert( StateFreeNode(4, test_a1.dest_car), 0, 0);
}

//utiliser pour générer les instances de tests aléatoirement
int rgenerator(int min,int max){

    std::random_device rd;

    std::mt19937 eng(rd());

    std::uniform_int_distribution<int> dist(min,max);
    return dist(eng);
}

//générer covoiturage simple et retourne un point de pick up et des coûts, fonction utilisée pour l'initialisation
//du covoiturage avec 3 points de pick-up et 1 point de drop-off
retour  covoit_gen(int src_ped, int src_car, int dest_ped, int dest_car, const Transport::Graph * gr)
{

        typedef AlgoMPR::CarSharingTest CurrAlgo;
	//structure qui sera retouurnée
	retour record;

        RLC::DFA dfa_car=RLC::car_dfa();
        RLC::DFA dfa_passenger=RLC::pt_foot_dfa();

        CurrAlgo::ParamType p(
              MuparoParams( gr, 5 ),
              AspectTargetParams( 4, dest_ped),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3));
   
        CurrAlgo cs(p);
        
        std::cout<<"Début covoiturage"<<endl;
        init_car_sharing<CurrAlgo>( &cs, gr, src_ped, src_car, dest_ped, dest_car, dfa_passenger, dfa_car);
        cs.run();
        std::cout<<"fin covoiturage"<<endl;

        int time=0;

        int drop_off = cs.get_source(4, dest_ped);
        int pick_up = cs.get_source(2, drop_off);
	int len1=cs.arrival(0, pick_up)-time;
	int len2=cs.arrival(1, pick_up)-time;
	//Récupération du coût entre l'origine voiture et le point de pick-up
	record.cost=len2;
	//Récupération du noeud de pick-up
	record.pick_up=pick_up;
	//Récupération du coût entre l'origine piéton et le point de pick-up
	record.cost_ped=len1;
	
	return record;
}


//test A*
//permet de rendre point_to_point générique
int pointToPoint(int src, int dest, const Transport::Graph * gr, bool car){
	int cost;
	RLC::DFA dfa_car=RLC::car_dfa();
    	RLC::DFA dfa_passenger=RLC::pt_foot_dfa();
	
	//Si l'on veut faire un point_to_point avec l'automate de la voiture 
	if (car){
	AlgoMPR::PtToPt * c1 =  point_to_point( gr, src, dest, dfa_car);
        c1->run();
        cost = c1->get_cost(0, dest);
	}
	//Si l'on veut faire un point_to_point avec l'automate du piéton
	else{
	AlgoMPR::PtToPt * c1 =  point_to_point( gr, src, dest, dfa_passenger);
        c1->run();
        cost = c1->get_cost(0, dest);
	}
	
	return cost;
}


//Permet de créer le fichier de récupération des instances de tests pour le covoiturage simple, on crée la première ligne de ce fichier qui va 
//indiquer le nom de chaque colonne
void create_file(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';
    
    fout << "src_car"<<';'<<"dest_car"<<";"<<"car_od_cost"<<';';

    fout << "src_ped"<<';'<<"dest_ped"<<";"<<"ped_od_cost"<<'\n';

    fout.close();
  }


//Permet d'écrire les valeurs des champs des instances de tests pour le covoiturage simple sur chaque ligne du fichier censé les récupérer
void save(string fileName,  structure_test test){

  
    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.src_car <<';'<< test.dest_car << ";" << test.car_od_cost <<';';

    fout << test.src_ped <<';'<< test.dest_ped << ";" << test.ped_od_cost <<'\n';
    
    fout.close();
    
}


//Permet de créer le fichier de récupération des instances de tests pour les covoiturages avec 3 piétons, on crée la première ligne de ce fichier qui va 
//indiquer le nom de chaque colonne
void create_file_v1(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';

    fout << "src_car"<<';'<<"dest_car"<<";"<<"car_od_cost"<<';';

    fout << "src_ped_1"<<';'<<"dest_ped_1"<<";"<<"ped_1_od_cost"<<';';

    fout << "src_ped_2"<<';'<<"dest_ped_2"<<";"<<"ped_2_od_cost"<<';';
 
    fout << "src_ped_3"<<';'<<"dest_ped_3"<<";"<<"ped_3_od_cost"<<'\n';
 
    fout.close();
  }

//Permet d'écrire les valeurs des champs des instances de tests pour les covoiturages avec 3 piétons sur chaque ligne du fichier censé les récupérer
void save_v1(string fileName,  structure_test_a1 test){


    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.src_car <<';'<< test.dest_car << ";" << test.car_od_cost <<';';

    fout << test.src_ped_1 <<';'<< test.dest_ped_1 << ";" << test.ped_1_od_cost <<';';

    fout << test.src_ped_2 <<';'<< test.dest_ped_2 << ";" << test.ped_2_od_cost << ';';

    fout << test.src_ped_3 <<';'<< test.dest_ped_3 << ";" << test.ped_3_od_cost << '\n';

    fout.close();

}

//Permet de créer le fichier de récupération des résultats de tests pour le covoiturages simple, on crée la première ligne de ce fichier qui va 
//indiquer le nom de chaque colonne
void create_file_covoit(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';

    fout << "point_pick_up"<<';'<<"point_drop_off"<<";"<<"coût_tot"<<';'<<"coût_algo_1"<<';'<<"coût_algo_2"<<';';

    fout << "coût_algo_3"<<';'<<"coût_algo_4"<<';'<<"coût_algo_5"<<';'<<"temps_execution"<<'\n';

    fout.close();
  }

//Permet d'écrire les valeurs des champs de la structure de résultat pour le covoiturage simple sur chaque ligne du fichier censé le récupérer
void save_covoit(string fileName,  structure_result_algo test){


    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.pick_up <<';'<< test.drop_off << ';' << test.cost_tot <<';';

    fout << test.cost_a1 <<';'<< test.cost_a2 << ';' << test.cost_a3 <<';' ;

    fout << test.cost_a4 <<';'<< test.cost_a5 << ';' << test.tps_exec <<'\n';

    fout.close();

}

//Permet de créer le fichier de récupération des résultats de tests pour le covoiturage avec 3 piétons, un seul point de pick-up et de drop-off,
//on crée la première ligne de ce fichier qui va indiquer le nom de chaque colonne
void create_file_covoit_v1(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';

    fout << "point_pick_up"<<';'<<"point_drop_off"<<";"<<"coût_tot"<<';'<<"coût_algo_1"<<';'<<"coût_algo_2"<<';';

    fout << "coût_algo_3"<<';'<<"coût_algo_4"<<';'<<"coût_algo_5"<<';'<<"coût_algo_6"<<';'<<"coût_algo_7"<<';'<<"temps_execution"<<'\n';

    fout.close();
  }


//Permet d'écrire les valeurs des champs de la structure de résultat pour le covoiturage avec 3 piétons, un seul point de pick-up et de drop-off, 
//sur chaque ligne du fichier censé le récupérer
void save_covoit_v1(string fileName,  structure_result_algo_v1 test){


    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.pick_up <<';'<< test.drop_off << ';' << test.cost_tot <<';';

    fout << test.cost_a1 <<';'<< test.cost_a2 << ';' << test.cost_a3 <<';' ;

    fout << test.cost_a4 <<';'<< test.cost_a5 <<  ';' << test.cost_a6 << ';' << test.cost_a7 << ';' << test.tps_exec <<'\n';
  
   

    fout.close();

}

//Permet de créer le fichier de récupération des résultats de tests pour le covoiturage avec 3 piétons, trois points de pick-up et un point de drop-off,
//on crée la première ligne de ce fichier qui va indiquer le nom de chaque colonne
void create_file_covoit_v2(string fileName){

    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout << "scenario_id" <<';';

    fout << "point_pick_up_1"<<';'<< "point_pick_up_2"<<';'<< "point_pick_up_3"<<';'<<"point_drop_off"<<";"<<"coût_tot"<<';'<<"coût_algo_1"<<';'<<"coût_algo_2"<<';';

    fout << "coût_algo_3"<<';'<<"coût_algo_4"<<';'<<"coût_algo_5"<<';'<<"coût_algo_6"<<';'<<"coût_algo_7"<<';'<<"temps_execution"<<'\n';

    fout.close();
  }

//Permet d'écrire les valeurs des champs de la structure de résultat pour le covoiturage avec 3 piétons, trois points de pick-up et un point de drop-off, 
//sur chaque ligne du fichier censé le récupérer
void save_covoit_v2(string fileName,  structure_result_algo_v2 test){


    ofstream fout;

    fout.open (fileName, ofstream::out | ofstream::app);

    fout <<test.scenario_id <<';';

    fout << test.pick_up_1 <<';'<< test.pick_up_2  <<';' << test.pick_up_3 <<';'<< test.drop_off << ';' << test.cost_tot <<';';

    fout << test.cost_a1 <<';'<< test.cost_a2 << ';' << test.cost_a3 <<';' ;

    fout << test.cost_a4 <<';'<< test.cost_a5 <<  ';' << test.cost_a6 << ';' << test.cost_a7 << ';' << test.tps_exec <<'\n';



    fout.close();

}

//lit le fichier csv contenant les instances de tests pour le covoiturage simple
void lecture(string filename, const Transport::Graph * gr, int number){

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");

    //Fichier csv qui contiendra les résulats du covoiturage
    string nom_res="/vagrant/projet/kool/analysis/resultat_bis.csv";

    create_file_covoit(nom_res);

    ifstream in(filename.c_str());

    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;

    while (getline(in,line) && i<=number+1){

      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());
	//std::cout<<"il a pas dit Bonjour"<<endl;
      if(i>=1)
      {
      	structure_test test;
      	std::cout<<"Aurevoir"<<endl;
      	test.scenario_id=std::atoi(vec.at(0).c_str());
      	test.src_car=std::atoi(vec.at(1).c_str());
     	test.dest_car=std::atoi(vec.at(2).c_str());
      	test.car_od_cost=std::atoi(vec.at(3).c_str());
      	test.src_ped=std::atoi(vec.at(4).c_str());
      	test.dest_ped=std::atoi(vec.at(5).c_str());
      	test.ped_od_cost=std::atoi(vec.at(6).c_str());

	//On lance ici la fonction du covoiturage simple
       	covoiturage_simple(nom_res, test, gr); 
	std::cout << test.src_car << std::endl;
      }
      i++;
    }
}

//lit le fichier csv contenant les instances de tests pour le covoiturage avec 3 piétons, un seul point de pick-up et de drop-off
void lecture_a1(string filename, const Transport::Graph * gr, int number){

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");

    //Fichier csv qui contiendra les résultats du covoiturage
    string nom_res="/vagrant/projet/kool/analysis/resultat_bis.csv";

    create_file_covoit_v1(nom_res);

    ifstream in(filename.c_str());

    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;

   
    while (getline(in,line) && i<=number+1){

      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());
     
      if(i>=1)
      {
      	structure_test_a1 test;
      	//On initialise la structure de test utilisé pour le covoiturage
     	test.scenario_id=std::atoi(vec.at(0).c_str());
      	test.src_car=std::atoi(vec.at(1).c_str());
      	test.dest_car=std::atoi(vec.at(2).c_str());
     	test.car_od_cost=std::atoi(vec.at(3).c_str());
      	test.src_ped_1=std::atoi(vec.at(4).c_str());
      	test.dest_ped_1=std::atoi(vec.at(5).c_str());
      	test.ped_1_od_cost=std::atoi(vec.at(6).c_str());
      	test.src_ped_2=std::atoi(vec.at(7).c_str());
      	test.dest_ped_2=std::atoi(vec.at(8).c_str());
     	test.ped_2_od_cost=std::atoi(vec.at(9).c_str());
      	test.src_ped_3=std::atoi(vec.at(10).c_str());
      	test.dest_ped_3=std::atoi(vec.at(11).c_str());
      	test.ped_3_od_cost=std::atoi(vec.at(12).c_str());
        //On lance ici la fonction du covoiturage avec 3 piétons, un seul point de pick-up et de drop-off
      	covoiturage_3ped_1dri_spick_sdrop(nom_res, test, gr);
        std::cout << test.src_car << std::endl;
      }
      i++;
    }
}

//lit le fichier csv contenant les instances de tests pour le covoiturage avec 3 piétons, trois points de pick-up et un point de drop-off
void lecture_a2(string filename, const Transport::Graph * gr, int number){

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    //Fichier qui contiendra les résultats du covoiturage
    string nom_res="/vagrant/projet/kool/analysis/resultat_bis.csv";

    create_file_covoit_v2(nom_res);

    ifstream in(filename.c_str());

    if (!in.is_open()) exit(0);

    vector< string > vec;
    string line;
    int i=0;

    while (getline(in,line) && i<=number+1){

      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());
        //std::cout<<"il a pas dit Bonjour"<<endl;
      if(i>=1)
      {
      	structure_test_a1 test;
      	//On initialise la structure de test
      	test.scenario_id=std::atoi(vec.at(0).c_str());
      	test.src_car=std::atoi(vec.at(1).c_str());
      	test.dest_car=std::atoi(vec.at(2).c_str());
      	test.car_od_cost=std::atoi(vec.at(3).c_str());
      	test.src_ped_1=std::atoi(vec.at(4).c_str());
      	test.dest_ped_1=std::atoi(vec.at(5).c_str());
     	test.ped_1_od_cost=std::atoi(vec.at(6).c_str());
      	test.src_ped_2=std::atoi(vec.at(7).c_str());
      	test.dest_ped_2=std::atoi(vec.at(8).c_str());
      	test.ped_2_od_cost=std::atoi(vec.at(9).c_str());
      	test.src_ped_3=std::atoi(vec.at(10).c_str());
      	test.dest_ped_3=std::atoi(vec.at(11).c_str());
      	test.ped_3_od_cost=std::atoi(vec.at(12).c_str());
        //On lance ici la fonction de covoiturage avec 3 piétons, trois points de pick-up et un point de drop-off
      	covoiturage_3ped_1dri_difpick_sdrop(nom_res, test, gr);
        std::cout << test.src_car << std::endl;
      }
      i++;
    }
}

//generer les instances de tests pour le covoiturage simple
void generer_instances(int number, int nb_nodes, const Transport::Graph * gr)
{
	
	int compt = 0;
	int min =0;
        int max = nb_nodes -1;
	bool valide=false;

	//On crée le fichier qui stockera les instances tests
	create_file("/vagrant/projet/kool/analysis/instances_bis.csv");
	//Tant que l'on a pas créé autant d'instances de tests qu'on le souhaite
	while (compt!= number){
        
        //Tant que l'on n'a pas généré une instance de tests vérifiant nos conditions
	while(!valide)
	{
		int src_car = rgenerator(min, max);
        	int src_ped = rgenerator(min, max);
		int cost_ocar_oped = pointToPoint(src_car, src_ped, gr, true);
		//Si la distance en terme de temps entre l'origine de la voiture et celle du piéton est inférieure à 1h
		if(cost_ocar_oped < 3600)
		{	
			int dest_car = rgenerator(min, max);
			int dest_ped = rgenerator(min, max);
			int cost_dcar_dped = pointToPoint(dest_car, dest_ped, gr, true);
			//Si la distance en terme de temps entre la destination de la voiture et celle du piéton est inférieure à 1h
			if(cost_dcar_dped < 3600)
			{
				int cost_car = pointToPoint(src_car, dest_car, gr, true);
        			int cost_ped = pointToPoint(src_ped, dest_ped, gr, false);
				//Si la distance entre l'origine du piéton et sa destination ainsi que la distance entre l'origine de la voiture et 		
				//sa destination sont supérieures à 0 seconde et à 10 minutes
				if (cost_car > 0 && cost_ped > 0 && cost_car > 600 && cost_ped > 600) 
				{	
					//on crée l'instance de test
					structure_test test;
					test.scenario_id =compt;
					compt++;
					test.src_car = src_car;
					test.src_ped = src_ped;
					test.dest_car = dest_car;
					test.dest_ped = dest_ped;
					test.car_od_cost = cost_car;
					test.ped_od_cost = cost_ped;
					//on l'enregistre dans le fichier csv
					save("/vagrant/projet/kool/analysis/instances_bis.csv", test);
					std::cout <<" ..... "<<"instance num: " << compt<< std::endl;
					valide=true;	
				}
        		}
		}
	}
	valide=false;
	}
}


//generer les instances de tests pour les covoiturages avec 3 piétons
void generer_instances_algo(int number, int nb_nodes, const Transport::Graph * gr)
{

        int compt = 0;
        int min =0;
        int max = nb_nodes -1;
        bool valide=false;
	//On crée le fichier qui va contenir les insatnces de tests
        create_file("/vagrant/projet/kool/analysis/instances_bis.csv");
        //Tant que l'on a pas créé autant d'instances de tests qu'on le souhaite
	while (compt!= number){

	//Tant que l'on n'a pas généré une instance de tests vérifiant nos conditions
        while(!valide)
        {
                int src_car = rgenerator(min, max);
                int src_ped_1 = rgenerator(min, max);
                int src_ped_2 = rgenerator(min, max);
		int src_ped_3 = rgenerator(min, max);
		int cost_ocar_oped_1 = pointToPoint(src_car, src_ped_1, gr, true);
		int cost_ocar_oped_2 = pointToPoint(src_car, src_ped_2, gr, true);
		int cost_ocar_oped_3 = pointToPoint(src_car, src_ped_3, gr, true);
		//Si la distance en terme de temps entre l'origine de la voiture et celle du piéton1, puis celle du piéton2 et enfin celle du piéton3 est inférieure à 1h
                if(cost_ocar_oped_1 < 3600 && cost_ocar_oped_2 < 3600 && cost_ocar_oped_3 < 3600)
                {
                        int dest_car = rgenerator(min, max);
                        int dest_ped_1 = rgenerator(min, max);
			int dest_ped_2 = rgenerator(min, max);
                        int dest_ped_3 = rgenerator(min, max);
			int cost_dcar_dped_1 = pointToPoint(dest_car, dest_ped_1, gr, true);
                        int cost_dcar_dped_2 = pointToPoint(dest_car, dest_ped_2, gr, true);
			int cost_dcar_dped_3 = pointToPoint(dest_car, dest_ped_3, gr, true);
			//Si la distance en terme de temps entre la destination de la voiture et celle du piéton1, 
			//puis celle du piéton2 et enfin celle du piéton3 est inférieure à 1h
			if(cost_dcar_dped_1 < 3600 && cost_dcar_dped_2 < 3600 && cost_dcar_dped_3 < 3600)
                        {
                                int cost_car = pointToPoint(src_car, dest_car, gr, true);
                                int cost_ped_1 = pointToPoint(src_ped_1, dest_ped_1, gr, false);
				int cost_ped_2 = pointToPoint(src_ped_2, dest_ped_2, gr, false);
                                int cost_ped_3 = pointToPoint(src_ped_3, dest_ped_3, gr, false);
				//Si la distance entre les origines des piétons et sa destination ainsi que la distance entre l'origine de la voiture et             
                                //sa destination sont supérieures à 0 seconde et à 10 minutes
				if (cost_car > 0 && cost_ped_1 > 0 && cost_ped_2 > 0 && cost_ped_3 > 0 && cost_car > 600 && cost_ped_1 > 600 && cost_ped_2 > 600 && cost_ped_3 > 600)
                                {
					//on crée l'insatnce de test
                                        structure_test_a1 test;
                                        test.scenario_id =compt;
                                        compt++;
                                        test.src_car = src_car;
                                        test.src_ped_1 = src_ped_1;
                                        test.src_ped_2 = src_ped_2;
					test.src_ped_3 = src_ped_3;
					test.dest_car = dest_car;
                                        test.dest_ped_1 = dest_ped_1;
					test.dest_ped_2 = dest_ped_2;
					test.dest_ped_3 = dest_ped_3;
                                        test.car_od_cost = cost_car;
					test.ped_1_od_cost = cost_ped_1;
					test.ped_2_od_cost = cost_ped_2;
					test.ped_3_od_cost = cost_ped_3;
                                        //on l'enregistre dans le fichier csv
                                        save_v1("/vagrant/projet/kool/analysis/instances_bis.csv", test);
                                        std::cout <<" ..... "<<"instance num: " << compt<< std::endl;
                                        valide=true;
                                }
                        }
                }
        }
        valide=false;
        }
}


//Fonction du covoiturage simple
void covoiturage_simple(string filename, structure_test test, const Transport::Graph * gr){
	
	structure_result_algo resultat;
    	typedef AlgoMPR::CarSharingTest CurrAlgo;

   	START_TICKING;

    	RLC::DFA dfa_car=RLC::car_dfa();
    	RLC::DFA dfa_passenger=RLC::pt_foot_dfa();

    	CurrAlgo::ParamType p(
              MuparoParams( gr, 5 ),
              AspectTargetParams( 4, test.dest_ped),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3));
    
    	CurrAlgo cs(p);
    
	std::cout<<"Début covoiturage"<<endl;
    	init_car_sharing<CurrAlgo>( &cs, gr, test.src_ped, test.src_car, test.dest_ped, test.dest_car, dfa_passenger, dfa_car);
	cs.run();
	std::cout<<"fin covoiturage"<<endl;
    	STOP_TICKING;

	int time=0;

    	int drop_off = cs.get_source(4, test.dest_ped);
    	int pick_up = cs.get_source(2, drop_off);

    	int len1=cs.arrival(0, pick_up)-time;
    	int len2=cs.arrival(1, pick_up)-time;
    	int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up); //temps entre pickup et le dropoff

    	int len4=cs.get_cost(3, drop_off); //on récupère le coût entre le dropof et la destination de la voiture
    	int len5=cs.arrival(4, test.dest_ped) - cs.arrival(4, drop_off); //te
	int tps_execution=RUNTIME;

	//On initialise les champs de la structure résultat
	resultat.scenario_id=test.scenario_id;
	resultat.drop_off=drop_off;
	resultat.pick_up=pick_up;
	resultat.cost_tot=len1+len2+2*len3+len4+len5;
	resultat.cost_a1=len1;
	resultat.cost_a2=len2;
	resultat.cost_a3=len3;
	resultat.cost_a4=len4;
	resultat.cost_a5=len5;
	resultat.tps_exec=tps_execution;

	//On enregistre les résultats dans le fichier csv
	save_covoit(filename, resultat);
}


//Fonction pour le covoiturage avce 3 piétons, un seul point de pick-up et un seul point de drop-off
void covoiturage_3ped_1dri_spick_sdrop(string filename, structure_test_a1 test, const Transport::Graph * gr){
        
	structure_result_algo_v1 resultat;
        typedef AlgoMPR::CarSharingTest CurrAlgo;

        START_TICKING;

        RLC::DFA dfa_car=RLC::car_dfa();
        RLC::DFA dfa_passenger=RLC::pt_foot_dfa();

        CurrAlgo::ParamType p(
              MuparoParams( gr, 9 ),
              AspectTargetParams( 8, test.dest_ped_1),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3));
        
	CurrAlgo cs(p);
        
	std::cout<<"Début covoiturage"<<endl;
        init_car_sharing_3ped_1driv_sdrop_spick<CurrAlgo>( &cs, gr, test, dfa_passenger, dfa_car);
	cs.run();
        std::cout<<"fin covoiturage"<<endl;
        STOP_TICKING;

        int time=0;

	
        int drop_off = cs.get_source(6, test.dest_ped_1);
	int pick_up = cs.get_source(4, drop_off);

	//Temps entre l'origine du piéton1 et le point de pick-up
        int len1_1=cs.arrival(0, pick_up)-time;
        
	//Temps entre l'origine du piéton2 et le point de pick-up
	int len1_2=cs.arrival(1, pick_up)-time;
 	
        //Temps entre l'origine du piéton3 et le point de pick-up
	int len1_3=cs.arrival(2, pick_up)-time;

        //Temps entre l'origine de la voiture et le point de pick-up
        int len2=cs.arrival(3, pick_up)-time;

        int len3=cs.arrival(4, drop_off) - cs.arrival(4, pick_up); //Temps entre pickup et le dropoff

        int len4=cs.get_cost(3, drop_off); //On récupère le coût entre le dropof et la destination de la voiture
        
	//Temps entre le point de drop-off et la destination de chaque piéton
	int len5_1=cs.arrival(6, test.dest_ped_1) - cs.arrival(6, drop_off);
        int len5_2=cs.arrival(7, test.dest_ped_2) - cs.arrival(6, drop_off);
	int len5_3=cs.arrival(8, test.dest_ped_3) - cs.arrival(6, drop_off);
	int tps_execution=RUNTIME;

	//On initialise la strcuture des résultats
        resultat.scenario_id=test.scenario_id;
        resultat.drop_off=drop_off;
        resultat.pick_up=pick_up;
        resultat.cost_tot=len1_1+len1_2+len1_3+len2+4*len3+len4+len5_1+len5_2+len5_3;
        resultat.cost_a1=len1_1;
        resultat.cost_a2=len1_2;
        resultat.cost_a3=len1_3;
        resultat.cost_a4=len2;
        resultat.cost_a5=len3;
	resultat.cost_a6=len4;
	resultat.cost_a7=len5_1;
	resultat.cost_a7=len5_2;
	resultat.cost_a7=len5_3;
        resultat.tps_exec=tps_execution;

	//On sauvegarde la structure des résultats dans le fichier csv
        save_covoit_v1(filename, resultat);
}


void covoiturage_3ped_1dri_difpick_sdrop(string filename, structure_test_a1 test, const Transport::Graph * gr){

        structure_result_algo_v2 resultat;
        typedef AlgoMPR::CarSharingTest CurrAlgo;
	int len1_1, len1_2, len1_3,len5_1, len5_2, len5_3;
        
        START_TICKING;

        RLC::DFA dfa_car=RLC::car_dfa();
        RLC::DFA dfa_passenger=RLC::pt_foot_dfa();

        CurrAlgo::ParamType p(
              MuparoParams( gr, 5 ),
              AspectTargetParams( 4, test.dest_ped_1),
              AspectPropagationRuleParams( SumPlusWaitCost, MaxArrival, 2, 0, 1),
              AspectPropagationRuleParams( SumCost, FirstLayerArrival, 4, 2, 3));
        
        CurrAlgo cs(p);
        
        std::cout<<"Début covoiturage"<<endl;
        init_car_sharing_3ped_1driv_sdrop_difpick<CurrAlgo>( &cs, gr, test, dfa_passenger, dfa_car);
        //On lance le covoiturage entre le second point de pick-up et l'origine du piéton qui n'a pas encore été récupéré
        cs.run();
        std::cout<<"fin covoiturage"<<endl;
        STOP_TICKING;

        int time=0;
	int dest, dest_2, dest_3;
        
	//Si le piéton restant était le piéton 1
	if(ped_rest==test.src_ped_1)
	{
		//La destination du covoiturage est celle du piéton 1 dans ce cas
		dest=test.dest_ped_1;
		dest_2=test.dest_ped_2;
		dest_3=test.dest_ped_3;
	}
	//Si le piéton restant était le piéton 2
	else if(ped_rest==test.src_ped_2)
	{
                //La destination du covoiturage est celle du piéton 2 dans ce cas
 		dest=test.dest_ped_2;
		dest_2= test.dest_ped_1;
		dest_3= test.dest_ped_3;
	}
	//Si le piéton restant était le piéton 3
	else if (ped_rest==test.src_ped_3)
	{
                //La destination du covoiturage est celle du piéton 3 dans ce cas
	 	dest=test.dest_ped_3;
		dest_2 = test.dest_ped_1;
		dest_3=test.dest_ped_2;
	}

        int drop_off = cs.get_source(4, dest);

        int pick_up = cs.get_source(2, drop_off);

	//Si le premier point de pick-up est celui du piéton 1
	if(node_pick_up1==test.src_ped_1)
        {
		//Temps entre l'origine du premier piéton récupéré et le premier point de pick-up
                len1_1=cost_pick_up1-time;
		//Si le second point de pick-up est celui du piéton 2
		if(node_pick_up2==test.src_ped_2)
		{
			 //Temps entre l'origine du second piéton récupéré et le second point de pick-up
                	len1_2=cost_pick_up2-time;
                 	//Temps entre l'origine du dernier  piéton récupéré et le troisième point de pick-up
			len1_3=cs.arrival(0, pick_up)-time;
        	}
                //Si le second point de pick-up est celui du piéton 3
		else if(node_pick_up2==test.src_ped_3)
		{	
			len1_3=cost_pick_up2-time;
                        len1_2=cs.arrival(0, pick_up)-time;
		}
	}
        //Si le premier point de pick-up est celui du piéton 2
	if(node_pick_up1==test.src_ped_2)
        {
                len1_2=cost_pick_up1-time;
                if(node_pick_up2==test.src_ped_1)
                {
                        len1_1=cost_pick_up2-time;
                        len1_3=cs.arrival(0, pick_up)-time;
                }
		else if(node_pick_up2==test.src_ped_3)
                {
			len1_3=cost_pick_up2-time;
                        len1_1=cs.arrival(0, pick_up)-time;
                }
        }
	//Si le premier point de pick-up est celui du piéton 3
	if(node_pick_up1==test.src_ped_3)
        {
                len1_3=cost_pick_up1-time;
                if(node_pick_up2==test.src_ped_1)
                {
                        len1_1=cost_pick_up2-time;
                        len1_2=cs.arrival(0, pick_up)-time;
                }
		else if(node_pick_up2==test.src_ped_2)
                {
                        len1_2=cost_pick_up2-time;
                        len1_1=cs.arrival(0, pick_up)-time;
                }
        }
	//Temps entre le second point de pick-up et le troisème point de pick-up
	int len2=cs.arrival(1, pick_up)-time;
        int len3=cs.arrival(2, drop_off) - cs.arrival(2, pick_up); //temps entre le troisième point de pickup et le drop-off

        int len4=cs.get_cost(3, drop_off); //on récupère le coût entre le drop-off et la destination de la voiture
	
	//Si la destination du piéton restant est celle du piéton 1
	if(dest==test.dest_ped_1)
	{
		//On récupère le temps entre le point de drop-off et la destination du piéton restant
		len5_1=cs.arrival(4, dest) - cs.arrival(4, drop_off);
		//On effectue un pointToPoint entre le point de drop-off du covoiturage et la destination de chaque piéton
		len5_2=pointToPoint(drop_off, test.dest_ped_2, gr, false);
        	len5_3=pointToPoint(drop_off, test.dest_ped_3, gr, false);
	}
        //Si la destination du piéton restant est celle du piéton 2
	if(dest==test.dest_ped_2)
        {
                len5_2=cs.arrival(4, dest) - cs.arrival(4, drop_off);
        	len5_1=pointToPoint(drop_off, test.dest_ped_1, gr, false);
        	len5_3=pointToPoint(drop_off, test.dest_ped_3, gr, false);
	}
        //Si la destination du piéton restant celle du piéton 3
	if(dest==test.dest_ped_3)
        {
                len5_3=cs.arrival(4, dest) - cs.arrival(4, drop_off);
        	len5_2=pointToPoint(drop_off, test.dest_ped_2, gr, false);
        	len5_1=pointToPoint(drop_off, test.dest_ped_1, gr, false);
	}
        int tps_execution=RUNTIME;

	int len1_1bis=cost_ped_pick_up1;
	int len1_2bis=cost_ped_pick_up2;
        
	//On initialise la structure contenant les résulats
	resultat.scenario_id=test.scenario_id;
        resultat.drop_off=drop_off;
        resultat.pick_up_1=pick_up1;
	resultat.pick_up_2=pick_up2;
	resultat.pick_up_3=pick_up;

	//Coût total du covoiturage
      	resultat.cost_tot=len1_1+len1_1bis+2*len1_2+len1_2bis+len1_3+3*len2+4*len3+len4+len5_1+len5_2+len5_3;
        resultat.cost_a1=len1_1;
        resultat.cost_a2=len1_2;
        resultat.cost_a3=len1_3;
        resultat.cost_a4=len2;
        resultat.cost_a5=len3;
        resultat.cost_a6=len4;
        resultat.cost_a7=len5_1;
        resultat.cost_a8=len5_2;
        resultat.cost_a9=len5_3;
        resultat.tps_exec=tps_execution;

        //On écrit les résultats dans le fichier csv
	save_covoit_v2(filename, resultat);
}


int main(int argc, char const *argv[])
{
	//On crée le graphe 
    	Transport::GraphFactory gf("/vagrant/projet/graph.txt-dump", false);
    	gf.setAll2();
    	const Transport::Graph * gr = gf.get();
   	
	//On génère les instances de tests pour le covoiturage simple
	generer_instances(nb_instances, gr->num_vertices(), gr);
   	std::cout<<"Je runne le main --------------------------"<<endl;
	//On lit le fichier csv contenant les instances de tests du covoiturage simple, on exécute le covoiturage simple
	//et on récupère les résultats dans un fichier csv
	lecture("/vagrant/projet/kool/analysis/instances_bis.csv", gr, nb_instances);
	
	//Pour les tests du covoiturage avec 3 piétons, trois points de pick-up et un point de drop-off

	//generer_instances_algo(nb_instances, gr->num_vertices(), gr);
	//std::cout<<"Je runne le main --------------------------"<<endl;
	//lecture_a2("/vagrant/projet/kool/analysis/instances_bis.csv", gr, nb_instances);
    	return 0;
}

