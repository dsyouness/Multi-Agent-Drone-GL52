/**
* Name: Drone Simulation
* Author: Group n°10
* Description: This model displays a Drone Simulation in 3D. It's about 2 defined missions for the drones :
*		-Helping people who get sick on the roads and cure them. 
*		-delivering packages to the connected buildings.
* 	And adding the communication aspect between drones that are close to each other to tell about there 
*	positions and identity.
* Tags: Drones, Multi-Agents, GL52 Project
*/


model drone_sim

global
{
	/**********************************
	 * Drone's global variables
	 *********************************/
	float step <- 20.0;
	//Number of delivery drones by default
	int nb_drone_delivery <- 4;
	//Number of ambulance drones by default
	int nb_drone_ambulance <- 4;
	//Minumum speed for a drone
	float min_speed <- 1.0 #km / #h;
	//Maximum speed for a drone
	float max_speed <- 5.0 #km / #h; 	
	//Define 50 experiment cycles
	int recompute_shortest_path <- 50;
	
	
	/*************************************************************************
	 * Import the shapefiles for GIS (Geographic Information System)
	 * That we have created to fill in our needs
	 *********************************************************************** */
	//Extract the buildings
	file shape_file_buildings <- file("../includes/building.shp");
	//Extract the roads
	file shape_file_roads <- file("../includes/road.shp");
	//Extract the bonds 
	file shape_file_bounds <- file("../includes/bounds.shp");
	/*Surround the city with bounds */
	geometry shape <- envelope(shape_file_bounds);
	/*Graph of the road network*/
	graph road_network;
	/*Map containing all the weights for the road network graph*/
	map<road,float> road_weights;
	bool save_shortest_paths <- false;
	bool load_shortest_paths <- false;
	string shortest_paths_file <- "../includes/shortest_paths.csv";
	bool memorize_shortest_paths <- true;
	string optimizer_type <- "Djikstra";
	
	
	
	
	/**********************************
	 * People's global variables
	 ***********************************/	
	//Number of people by default
	int nb_people <- 15;
	//Define the speed of people agents
	float speed_people <- 0.5 #km/#h;
	//The probability of getting sick or having an accident
	float proba_accident <- 1/1000;
	//Declare a list of locations where people who got sick or had an accident are located
	list<point> list_of_location_accident_victims <- [];
	//Declare a list of people who got sick or had an accident
	list<people> accident_victims <- [];
	//Declaring a counter of  people who got sick or had an accident
	int nb_of_accident min:0 max:15;
	//Declaring a counter of healed people
	int nb_of_healed_people <- nb_people min:0 max:15;
	


	/***********************************************
	 ********Initialization of the simulation********
	 *************************************************/
	init
	{
		/*Initialization and creating of building agents:
		 * We have defined 4 types of connected buildings:
		 * 	- Connected (those who are going to receive the package)
		 * 	-Docking station (from where delivery drones will get the package that should be delivered to Connected)
		 * 	-Charging station (where the drones are going to charge there batteries)
		 * 	-Hospital (from where the ambulance drone with get maximum first aid kit which is defined by 3 kits)
		 */
		create building_connected from: shape_file_buildings with: [type::string(read ("NATURE"))] {
			if type="Connected" {
				color <- #gamaorange ;
			}
			if type="Docking" {
				color <- #fuchsia ;
			}
			if type="Charging" {
				color <- #blue ;
			}
			if type="Hospital" {
				color <- #red ;
			}
		}
		
		/*List of buildings*/
		list<building_connected>  connected_buildings <- building_connected  where (each.type="Connected") ;
		list<building_connected>  docking_buildings <- building_connected  where (each.type="Docking") ;
		list<building_connected>  charging_buildings <- building_connected  where (each.type="Charging") ;
		list<building_connected>  hospital_buildings <- building_connected  where (each.type="Hospital") ;
		
		
		
			
		/*************************************************************************************
		 * ************************Initialization road agents:******************************
		 * 	-Every road should be defined are a graph
		 * 	-Compute the shortest path to a traget using a CSV File in "includes" file
		 **************************************************************************************/
		create road from: shape_file_roads ;
		
		//Each road is represented by a graph
		road_network <- as_edge_graph(list(road));
		
		
		/*4 type of optimizer can be used for the shortest path computation:
		 *    - Djikstra: the default one - ensure to find the best shortest path - compute one shortest path at a time (by default, memorise the shortest path found)
		 * 	  - Bellmann: ensure to find the best shortest path - compute one shortest path at a time (by default, memorise the shortest path found)
		 * 	  - AStar: do not ensure to find the best shortest path - compute one shortest path at a time (by default, memorise the shortest path found)
		 *    - Floyd Warshall: ensure to find the best shortest path - compute all the shortest pathes at the same time (and keep them in memory)
		 */
		/*Compute the shortest path to a target */		
		//allows to choose the type of algorithm to use compute the shortest paths
		road_network <- road_network with_optimizer_type optimizer_type;
		
		//Allows to define if the shortest paths computed should be memorized (in a cache) or not
		road_network <- road_network use_cache memorize_shortest_paths;
		
		//Computes all the shortest paths, puts them in a matrix, then saves the matrix in a file
		if save_shortest_paths {
			matrix ssp <- all_pairs_shortest_path(road_network);
			save ssp type:"text" to:shortest_paths_file;
			
		//Loads the file of the shortest paths as a matrix and uses it to initialize all the shortest paths of the graph
		} else if load_shortest_paths {
			road_network <- road_network load_shortest_paths matrix(file(shortest_paths_file));
		}

		
		
		
		
		
		/**************************************************************
		 * *************Initialization of drone agents****************
		 **************************************************************/
		create delivery_drone number: nb_drone_delivery{
			speed <- min_speed + rnd (max_speed - min_speed) ;			
			charging_place <- one_of(charging_buildings) ;
			spawning_place <- one_of(docking_buildings) ;
			delivery_place <- one_of(connected_buildings) ;
			objective <- "delivering";
			location <- any_location_in (spawning_place);
			the_target <- location;
		}
		
		create ambulance_drone number: nb_drone_ambulance {
			speed <- min_speed + rnd (max_speed - min_speed) ;
			spawning_place <- one_of(hospital_buildings) ;
			charging_place <- one_of(charging_buildings) ;
			objective <- "kit_taken";
			location <- any_location_in (spawning_place);
			the_target <- location;
		}
		
		
		
		
		/**************************************************************
		 * *************Initialization of people agents****************
		 **************************************************************/
		create people number:nb_people{			
			people_target  <- any_location_in(one_of (road)) ;
			location <- any_location_in (one_of(road));
			source <- location;
			speed <- speed_people;
		}
		
		/*List of infected people */
		list<people> accident_victims <- people  where (each.hadAccident="true") ;		
	}	
}








/*****************************************************
 ***************Defining agents********************
 * 	- Building (Parent agent):
 * 		* Connected building (Child agent)
 * 	- Road 
 *	- Drone (Parent agent) :
 * 		* Delivery drone (Child agent)
 * 		* Ambulance drone (Child agent)
 *	- People 
 *****************************************************/

/*Building Agent*/
species building {
	/*Attributes*/
	//Color of buildings by default
	rgb color <- #gray ;
	//Height of buildings (randomized)
	float height <- 10#m + rnd(30) #m;
	//Aspect is used to define a way to draw the "Building" agents
	aspect geom {
        draw shape color: color depth: height;
    }
}
/* Connected building (Child agent)*/
species building_connected parent: building{
	string type ;
}


/********************************************************************************************/


/*Road Agent*/
species road  {
	geometry display_shape <- shape + 2.0;
	//Aspect is used to define a way to draw the "Road" agents
    aspect geom {
        draw display_shape color: #black depth: 3.0;
    }
}


/********************************************************************************************/

/*People Agent*/
species people skills:[moving]{
	/*Attributes*/
	//Boolean hadAccident is true when a person is an accident victim
	bool hadAccident <- false;
	//Target where the person is heading to
	point people_target;
	//Location of a person
	point source;

	
	/*Actions and Behaviours */
	
	//Moving behaviour : when the person is ok or healed , he is moving everywhere on the road
	reflex move when: hadAccident = false{	 	
	 	do wander on:road_network;
	}
	
	//haveAccident behaviour : when the person is ok or healed, there is a probability of getting into an accident
	reflex haveAccident when: hadAccident = false{ 	
			 if flip(proba_accident) {
				 hadAccident <- true;
				 nb_of_accident <- nb_of_accident + 1;
				 nb_of_healed_people <- nb_of_healed_people - 1;
				 add (location) to: list_of_location_accident_victims;
				 add (self) to: accident_victims;
		} 
	}

	//When a person get in an accident, he stops moving
	reflex stop_moving when: hadAccident = true {
		people_target <- nil;
	}
	
	//Aspect is used to define a way to draw the "People" agents
	aspect People3D {
			 draw pyramid(15) at: {location.x,location.y,location.z + 2} color:hadAccident ? #red : #white;
			 draw sphere(5) at: {location.x,location.y,location.z + 15} color:hadAccident ? #red : #white;
	 	 }
}


/********************************************************************************************/


/*Drone Agent*/
species drone skills:[moving]  {
	
	/*Attributes*/
	//Energy of a drone (level of battery)
	float energy_drone <- rnd(50.0,100.0) min:0.0 max:100.0 update:energy_drone-0.2;
	//Color of the agent
	rgb color<-nil;
	//Charging place
	building charging_place <- nil ;
	//Place where the agent going to spawn
	building spawning_place <- nil ;
	//Drone's objective
	string objective;
	//Path to follow by drones 
	path path_to_follow <- nil;
	//Point where the agent going to fly to
	point the_target <- nil ;
	
	
	/*Actions and Behaviours */
	//Communication action
	action communicate {
		
		//A drone ask another delivery_drone about its identity and position
		ask (delivery_drone) closest_to self{
			write 'I am drone ' + name + ' at the position : ' + location;
			write'#######################################################';
		}
		//A drone ask another ambulance_drone about its identity and position
		ask(ambulance_drone) closest_to self{
			write 'I am drone ' + name + ' at the position : ' + location;
			write'#######################################################';
		}
	}
	 
	//Moving behaviour : when the target is not null , the drone is moving towards it after computing the short path
	reflex move when: the_target != nil {
		if (path_to_follow = nil or every(recompute_shortest_path) ) {
			path_to_follow <- road_network path_between (location::the_target);
			write name + " has computed a new shortest path";
		} else {
			do follow path: path_to_follow;
		}
		do communicate;	
	}
	 
}


/*Delivery Drone Agent*/
species delivery_drone parent: drone{
	/*Attributes */
	//Ambulance drone color
	rgb color <- #aqua ;
	//Charging place
	building delivery_place <- nil ;//green
	//Spawning place
	building spawning_place <-nil; //olive
	//Number of deliveries made
	int nb_delivery;
	
	
	/*Actions and Behaviours */
	
	/*Delivering behavior : when the objective is set to delivering, the drone will carry a package to a specified connected building */
	reflex go_deliver when: self.the_target=self.location and objective = "delivering"{
		the_target <- any_location_in (delivery_place);
		delivery_place <- one_of(building_connected  where (each.type="Connected"));
		write self.name + " : I have a package delivery to " + delivery_place.name + ' from ' + spawning_place.name ;
		objective <- "take" ; 
	}
	
	/*Taking a package behavior : when the objective is set to take, the drone will go to docking station to take a package */
	reflex take_package when: self.the_target=self.location and objective="take"{
		write self.name + " : I've delivered a package to " + delivery_place.name;
		the_target <- any_location_in (spawning_place);
		spawning_place <- one_of(building_connected  where (each.type="Docking")) ;
		self.nb_delivery <- self.nb_delivery +1 ;
		objective <- "delivering" ;
	}
	
	/*Go to charge behavior : when the level of battery is lower than 25% , the drone will go to the nearest charging station */
	reflex go_charge when: energy_drone<= 25.0 and (objective="delivering" or objective="take") {
		write self.name + ' : I need to charge . Im on 15% energy';
		objective <- "charging" ;
		the_target <- any_location_in(charging_place);	
	}
	
	/*Charging behavior : After getting charged, the drone will continue his main objective which is delivering*/
	reflex charge when: self.the_target=self.location and objective="charging"{
		self.energy_drone <- 100.0;
		write self.name + ' : Im fully charged';
		objective <- "delivering" ;
	}
	
	//Aspect is used to define a way to draw the "Delivery drone's" agents
	aspect sphereDel3D{
        draw box(6,6,6) at: {location.x,location.y,location.z + 30} color: #aqua;
		draw square(15) at: {location.x,location.y,location.z + 35.5}  color: #aqua;
		draw sphere(2) at: {location.x + 7.5 ,location.y + 7.5,location.z + 35.5} color: #black;
		draw sphere(2) at: {location.x - 7.5 ,location.y - 7.5,location.z + 35.5} color: #black;
		draw sphere(2) at: {location.x-7.5,location.y + 7.5,location.z + 35.5} color: #black;
		draw sphere(2) at: {location.x + 7.5 ,location.y-7.5,location.z + 35.5} color: #black;	
	}

}


/*Ambulance Drone Agent*/
species ambulance_drone parent: drone{
	/*Attributes */
	//Ambulance drone color
	rgb color <- #chocolate ;
	//First aid kits
	int maxHealing <- 3 min: 0;
	
	/*Actions and Behaviours */

	/*Go to heal victims Behavior : when the ambulance drone have enough First aid kits on him and his objective is healing, it will go to heal a victim on the road*/
	reflex go_healVictims when: self.the_target=self.location and maxHealing > 0 and objective = "healing"{
		loop l over: list_of_location_accident_victims{
			the_target <- l;
		} 
		objective <- "do_heal";
		remove the_target from: list_of_location_accident_victims;
	}
	
	/*heal victims Behavior : the drone heals the victims and after every healing action, he will decrease the number of First aid kits by one*/
	reflex healVictims when: self.the_target=self.location and objective="do_heal"{
		loop x over: accident_victims{
			if (the_target = x.location) {
				x.hadAccident <- false;
				maxHealing <- maxHealing - 1;
				nb_of_accident <- nb_of_accident - 1;
				nb_of_healed_people <- nb_of_healed_people + 1 ;
			}
		}
		objective <- "healing";
	}
	
	/*Take kit behavior: when the ambulance drone have no more kits he will go to hospital to take three of them */
	reflex take_kit when: maxHealing = 0 and (objective="healing" or objective = "take_kit") {
		objective <- "kit_taken" ;
		the_target <- any_location_in(spawning_place);
		spawning_place <- one_of(building_connected  where (each.type="Hospital")) ;
	}
	
	/*Kit taken behavior: after taking the kits, he will continue his first priority which is healing victims */
	reflex kit_taken when: self.the_target=self.location and objective="kit_taken"{
		maxHealing <- 3 ;
		objective <- "healing";
	}
	
	
	/*Go to charge behavior : when the level of battery is lower than 25% , the drone will go to the nearest charging station */
	reflex go_charge when: energy_drone<= 25 and (objective="healing" or objective = "take_kit") {
		objective <- "charging" ;
		the_target <- any_location_in(charging_place);	
	}
	
	
	/*Charging behavior : After getting charged, the drone will continue his main objective which is healing*/
	reflex charge when: self.the_target=self.location and objective="charging"{
		self.energy_drone <-  100.0;
		objective <- "healing" ;
	}
	
	//Aspect is used to define a way to draw the "Delivery drone's" agents
	aspect sphereAmb3D{
        draw box(6,6,6) at: {location.x,location.y,location.z + 30} color: #chocolate;
		draw square(15) at: {location.x,location.y,location.z + 35.5}  color: #chocolate;
		draw sphere(2) at: {location.x + 7.5 ,location.y + 7.5,location.z + 35.5} color: #black;
		draw sphere(2) at: {location.x - 7.5 ,location.y - 7.5,location.z + 35.5} color: #black;
		draw sphere(2) at: {location.x-7.5,location.y + 7.5,location.z + 35.5} color: #black;
		draw sphere(2) at: {location.x + 7.5 ,location.y-7.5,location.z + 35.5} color: #black;	
	}
		
}





/**************************************
 ********Runnable experiment********
 ***************************************/

experiment Drone_Simulation3D type: gui {
	parameter "Shapefile for the buildings:" var: shape_file_buildings category: "GIS" ;
	parameter "Shapefile for the roads:" var: shape_file_roads category: "GIS" ;
	parameter "Shapefile for the bounds:" var: shape_file_bounds category: "GIS" ;
	parameter "Number of delivery drones' agents" var: nb_drone_delivery category: "Drone" ;
	parameter "Type of optimizer" var: optimizer_type among: ["Djikstra", "AStar", "Bellmann", "Floyd Warshall"];
	parameter "Computed all the shortest paths and save the results" var: save_shortest_paths;
	parameter "Load the shortest paths from the file" var: load_shortest_paths;
		
	output {
		display Simulation type:opengl {
			image "../images/grass.png";
			species road aspect:geom;
            species building_connected aspect:geom transparency: 0.4;
			species delivery_drone aspect: sphereDel3D ;
			species ambulance_drone aspect: sphereAmb3D ;
			species people aspect: People3D ;
			
		}
		display Chart_energy {
			chart "Level of energy of drones" type: histogram  {
				data "drone_delivery 0" value: (delivery_drone at 0).energy_drone color: #green;
				data "drone_delivery 1" value: (delivery_drone at 1).energy_drone color: #red;
				data "drone_delivery 2" value: (delivery_drone at 2).energy_drone color: #aqua;
				data "drone_delivery 3" value: (delivery_drone at 3).energy_drone color: #blue;
				data "drone_ambulance 0" value: (ambulance_drone at 0).energy_drone color: #purple;
			}
		}
		
		display Chart_nb_delivery {
			chart "Number of deliveries made" type:pie position:{0,0} size:{0.5,0.5} {
				data "drone_delivery 0" value: (delivery_drone at 0).nb_delivery color: #green;
				data "drone_delivery 1" value: (delivery_drone at 1).nb_delivery color: #red;
				data "drone_delivery 2" value: (delivery_drone at 2).nb_delivery color: #aqua;
				data "drone_delivery 3" value: (delivery_drone at 3).nb_delivery color: #blue;
			}
		
		}
		display Chart_accident_victims type: java2D{
			chart "Accident" type:series {
				data "Number of accidents" value: nb_of_accident color: # yellow marker: false style: line;
				data "Number of healed people" value: nb_of_healed_people color: # red marker: false style: line;
			}
		
		}
	}
}
