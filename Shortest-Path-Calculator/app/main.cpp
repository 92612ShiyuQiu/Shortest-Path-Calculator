// main.cpp
//
// ICS 46 Spring 2018
// Project #5: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.

#include "RoadSegment.hpp"
#include "InputReader.hpp"
#include "Trip.hpp"
#include "RoadMapWriter.hpp"
#include "RoadMapReader.hpp"
#include "RoadMap.hpp"
#include "TripReader.hpp"
#include "TripMetric.hpp"
#include <math.h>
#include <iostream>
#include <iomanip>

std::function<double(const RoadSegment&)> shortestDistance = [](RoadSegment rs){return rs.miles;};
std::function<double(const RoadSegment&)> shortestTime = [](RoadSegment rs){return rs.miles/rs.milesPerHour;};

std::vector<int> pv_ordered_ints(std::map<int, int> pv, int start, int end)
{
	std::vector<int> ordered_vertices;
	ordered_vertices.push_back(end);
	while (true)
	{
		int last = ordered_vertices[ordered_vertices.size()-1];
		ordered_vertices.push_back(pv.at(last));
		if (ordered_vertices.back() == start)
		{
			break;
		}
	}
	std::reverse(ordered_vertices.begin(), ordered_vertices.end());
	return ordered_vertices;
}

int main()
{
	InputReader cin_reader = InputReader(std::cin);
	RoadMapReader map_reader = RoadMapReader();
	RoadMap real_map = map_reader.readRoadMap(cin_reader);
	TripReader trip_reader = TripReader();
	std::vector<Trip> trips = trip_reader.readTrips(cin_reader);
	
	std::vector<std::pair<int, std::map<int, int>>> trip_pv;
	for (int i = 0; i < trips.size(); ++i)
	{
		if (trips[i].metric == TripMetric::Time)
		{
			std::map<int, int> pv = real_map.findShortestPaths(trips[i].startVertex, shortestTime);
			trip_pv.push_back(std::pair<int, std::map<int, int>>(i, pv));
		}
		if (trips[i].metric == TripMetric::Distance)
		{
			std::map<int, int> pv = real_map.findShortestPaths(trips[i].startVertex, shortestDistance);
			trip_pv.push_back(std::pair<int, std::map<int, int>>(i, pv));
		}
	}

	for (int b = 0; b < trips.size(); ++b)
	{
		std::vector<int> ordered_vertices = pv_ordered_ints(trip_pv[b].second, trips[b].startVertex, trips[b].endVertex);
		std::string choice;
		std::string inside_paren;
		std::string total_line = "Total ";
		if (trips[b].metric == TripMetric::Time)
		{
			double cost = 0.0;
			choice = "driving time";
			total_line += "time: ";
			std::string start_location = real_map.vertexInfo(trips[b].startVertex); 
			std::string end_location = real_map.vertexInfo(trips[b].endVertex);
			std::cout<<"Shortest "<<choice<<" from "<<start_location<<" to "<<end_location<<std::endl;
			std::cout<<"  Begin at "<<start_location<<std::endl;
			for (int c = 1; c < ordered_vertices.size(); ++c)
			{
				double edge_miles = real_map.edgeInfo(ordered_vertices[c-1], ordered_vertices[c]).miles;
				double edge_speed = real_map.edgeInfo(ordered_vertices[c-1], ordered_vertices[c]).milesPerHour; 
				double edge_hour = edge_miles/edge_speed;
				double edge_sec = edge_hour*3600.0;
				cost += edge_sec;
				std::cout<<"  Continue to "<<real_map.vertexInfo(ordered_vertices[c])
							 <<" ("<<std::setprecision(3)<<edge_miles<<" miles @ "
							 <<std::setprecision(3)<<edge_speed<<" = ";
				if (edge_hour > 1)
				{
					int hours = static_cast<int>(edge_hour);
					double left_hour = edge_hour - hours;
					double left_hour_to_sec = left_hour*3600;
					int left_hour_sec_to_min = static_cast<int>(floor(left_hour_to_sec/60.0));
					double sec_no_min = left_hour_to_sec - left_hour_sec_to_min*60.0;
					std::cout<<hours<<" hours "<<left_hour_sec_to_min<<" mins "<<std::setprecision(3)<<sec_no_min<<" secs)"<<std::endl;
				}
				else
				{
					if (edge_sec > 60.0)
					{
						int edge_min = static_cast<int>(floor(edge_sec/60.0)); 
						double left_sec = edge_sec - (edge_min*60.0);
						std::cout<<std::setprecision(3)<<edge_min<<" mins "
								 <<std::setprecision(3)<<left_sec<<" secs)"<<std::endl;
					}
					else
					{
						std::cout<<std::setprecision(3)<<edge_sec<<" secs)"<<std::endl;
					}
				}
			}

			double total_sec = cost;
			if (total_sec > 60.0)
			{
				int total_min = static_cast<int>(floor(total_sec/60.0));
				double total_left_sec = total_sec - (total_min*60.0);
				std::cout<<total_line<<std::setprecision(3)<<total_min<<" mins "
						 <<total_left_sec<<" secs"<<std::endl<<std::endl;
			}
			else
			{
				std::cout<<total_line<<std::setprecision(3)<<total_sec<<" secs"<<std::endl<<std::endl;
			}
		}

		if (trips[b].metric == TripMetric::Distance)
		{
			double cost = 0.0;
			choice = "distance";
			total_line += "distance: ";
			std::string start_location = real_map.vertexInfo(trips[b].startVertex); 
			std::string end_location = real_map.vertexInfo(trips[b].endVertex);
			std::cout<<"Shortest "<<choice<<" from "<<start_location<<" to "<<end_location<<std::endl;
			std::cout<<"  Begin at "<<start_location<<std::endl;
			for (int c = 1; c < ordered_vertices.size(); ++c)
			{
				double edge_miles = real_map.edgeInfo(ordered_vertices[c-1], ordered_vertices[c]).miles;
				cost += edge_miles;
				std::cout<<"  Continue to "<<real_map.vertexInfo(ordered_vertices[c])
						 <<" ("<<std::setprecision(3)<<edge_miles<<" miles)"<<std::endl;
			}
			double total_distance = cost;
			std::cout<<total_line<<std::setprecision(3)<<total_distance<<" miles"<<std::endl<<std::endl;
		}
	}
	
    return 0;
}

