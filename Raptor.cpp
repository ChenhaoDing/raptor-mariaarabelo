#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <iomanip>  // for setw
#include "Raptor.h"

Raptor::Raptor(const std::unordered_map<std::string, Stop>& stops,
               const std::unordered_map<std::pair<std::string, std::string>, Route, pair_hash>& routes,
               const std::unordered_map<std::string, Trip>& trips,
               const std::unordered_map<std::pair<std::string, std::string>, StopTime, pair_hash>& stop_times)
                : stops_(stops), routes_(routes), trips_(trips), stop_times_(stop_times) {
  k = 1;
  initializeData();
}

// TODO: move it to Parser class
void Raptor::initializeData() {
  // Initialize footpaths
  for (auto& [id, stop] : stops_) {
    for (const auto& [other_id, other_stop]: stops_) {
      if (id != other_id) {
        stop.footpaths[other_id] = {other_id, Utils::getDuration(stop.coordinates, other_stop.coordinates)};
      }
    }
  }

  // Associate stop_times to trips
  for (auto& [ key, stop_time] : stop_times_) {
    auto& [trip_id, stop_id] = key;
    trips_[trip_id].stop_times.push_back(&stop_time);
  }

  // Order each trip's stop_times
  for (auto& [id, trip] : trips_) {
    std::sort(trip.stop_times.begin(), trip.stop_times.end(), [](const StopTime* a, const StopTime* b) {
      return a->stop_sequence < b->stop_sequence;
    });
  }

  // Associate trips to routes
  for (auto& [id, trip] : trips_){
    auto route_key = std::make_pair(trip.route_id, trip.direction_id);
    routes_[route_key].trips.push_back(&trip);
  }

  // Order each route's trip by earliest to latest arrival time
  for (auto& [id, route] : routes_ ) {
    std::sort(route.trips.begin(), route.trips.end(), [&](const Trip *a, const Trip *b) {
      return Utils::timeToSeconds(a->stop_times.front()->arrival_time) < Utils::timeToSeconds(b->stop_times.front()->arrival_time);
    });
  }

  // Associate stops to routes
  for (auto& [key, route] : routes_){
    // If there is no trip associated to this route, skip the route
    if (route.trips.empty()) continue;

    // Find the route's trip that has the most stops
    Trip* largest_trip = route.trips[0];
    for (size_t i = 1; i<route.trips.size(); i++){
      Trip* trip = route.trips[i];
      if (trip->stop_times.size() > largest_trip->stop_times.size()){
        largest_trip = trip;
      }
    }

    // A route stops' order will be the same as the routes' largest_trip stops' order, because it has the most stops
    for (const auto& stop_time: largest_trip->stop_times){
      route.stops.push_back(&stops_[stop_time->stop_id]);
    }
  }

  for (auto& [key, stop_time] : stop_times_){
    auto& [trip_id, stop_id] = key;
    // Associate stop_times to stops
    stops_[stop_id].stop_times.push_back(&stop_time);

    // Associate routes to stops
    stops_[stop_id].routes_keys.emplace_back(trips_[trip_id].route_id, trips_[trip_id].direction_id);
  }

  // Order each stop's stop_times by earliest to latest departure time
  for (auto& [id, stop] : stops_){
    std::sort(stop.stop_times.begin(), stop.stop_times.end(), [&](const StopTime *a, const StopTime *b) {
      return Utils::timeToSeconds(a->departure_time) < Utils::timeToSeconds(b->departure_time);
    });
  }

}

std::vector<std::vector<JourneyStep>> Raptor::findJourneys(const Query &query) {

  std::cout << "Finding journeys from " << stops_[query.source_id].stop_name << " to " << stops_[query.target_id].stop_name
            << " departing at " << query.departure_time << std::endl << std::endl;

  std::unordered_set<std::string> marked_stops;

  // Initialization of the algorithm
  for (const auto& [id, stop]: stops_) {
    min_arrival_time[id] = std::vector<StopInfo>(1, {INF, "-1", "-1"});
  }
  min_arrival_time[query.source_id][0].min_arrival_time = Utils::timeToSeconds(query.departure_time);
  marked_stops.insert(query.source_id);
  k = 1;

  while (true) {
//    std::cout << std::endl << "Round " << k << std::endl << std::endl;

    // 1st: set an upper bound
    for (const auto& [id, stop]: stops_){
      min_arrival_time[id].push_back(min_arrival_time[id][k-1]);
    }

    // Accumulate routes serving marked stops from previous round
    // ((route_id, direction_id), stop_id)
    // TODO: would it be worth it to change to unordered_set?
    std::set<std::pair<std::pair<std::string, std::string>, std::string>> routes_stops_set;
    // For each marked stop p
    auto marked_stop_id = marked_stops.begin();
    while (marked_stop_id != marked_stops.end()) {

      // For each route r serving p
      for (const auto &route_key: stops_[*marked_stop_id].routes_keys) {

        // Iterate over all stops in the route
        bool already_has_point = false;
        for (Stop* stop: routes_[route_key].stops) {
          std::string stop_id = stop->stop_id;

          // Check if a point p' is already in the list
          auto existing_entry = routes_stops_set.find({route_key, stop_id});
          if (existing_entry != routes_stops_set.end()) {
            already_has_point = true;

            auto it_marked = std::find_if(routes_[route_key].stops.begin(), routes_[route_key].stops.end(),
                                          [&](Stop* s) { return s->stop_id == *marked_stop_id; });
            auto it_stop = std::find_if(routes_[route_key].stops.begin(), routes_[route_key].stops.end(),
                                        [&](Stop* s) { return s->stop_id == stop_id; });

            if (it_marked < it_stop) { // if marked_p comes before p'
              routes_stops_set.erase(existing_entry);
              routes_stops_set.insert({route_key, *marked_stop_id});
            } // if marked_p does not come before p', we do not add it to the set and leave the p' entry
          }
        }

        if (!already_has_point)  // The route does not have a point in the list
          routes_stops_set.insert({route_key, *marked_stop_id});
      }

      // Unmark p
      marked_stop_id = marked_stops.erase(marked_stop_id); // Next valid stop
    }

    // 2nd: Traverse each route
    auto route_stop = routes_stops_set.begin();
//    std::cout << "In round " << k << " we have " << routes_stops_set.size() << " routes_stops pairs to iterate trough." << std::endl;

//    for (auto& [route_key, stop_id]: routes_stops_set){
//      std::cout << "[" << route_key.first << ", " << route_key.second << "], " << stop_id
//      << " (from " << stops_[stop_id].stop_name << ")" << std::endl;
//    }

    while (route_stop != routes_stops_set.end()) {
      const auto& [route_key, p_stop_id] = *route_stop;
      const Route& route = routes_[route_key];

//      std::cout << std::endl;
//      std::cout << "Traversing route " << route.route_id << " d " << route.direction_id << " starting from " << stops_[p_stop_id].stop_name << std::endl ;
//      for (Stop *stop : route.stops){
//        std::cout << stop->stop_id << " " << stop->stop_name << std::endl;
//      }

      // TODO: what is the complexity of this?
      auto stop_it = std::find_if(route.stops.begin(), route.stops.end(),
                                         [&](Stop* stop) { return stop->stop_id == p_stop_id; });

      // Remaining stops to be traversed on the route:
//      std::cout << "Remaining stops to be traversed on route" << route_key.first << ", " << route_key.second << ": " << std::endl;
//      for (auto it = stop_it; it != route.stops.end(); ++it) {
//        std::cout << (*it)->stop_id << "-" << (*it)->stop_name << " ";
//      }
//      std::cout << std::endl << std::endl << std::endl;

      // For each stop pi on this route, try to find the earliest trip (et) that can be taken
      for (auto it = stop_it; it != route.stops.end(); ++it) {
        Stop* pi_stop = *it;
        std::string pi_stop_id = pi_stop->stop_id;
        std::string et_id = "-1";

        // Find the earliest trip in route r that can be caught at stop pi in round k
        for (StopTime* stop_time : stops_[pi_stop_id].stop_times){
          // Check if the stop_time is from a trip that belongs to the route being traversed
          std::string stop_time_trip_id = stop_time->trip_id;
          if ((trips_[stop_time_trip_id].route_id == route.route_id) && (trips_[stop_time_trip_id].direction_id == route.direction_id)
            && (Utils::timeToSeconds(stop_time->departure_time) >= min_arrival_time[pi_stop_id][k-1].min_arrival_time)
            // TODO: Can/Should I add this optimization here?
            && (Utils::timeToSeconds(stop_time->departure_time) < min_arrival_time[query.target_id][k].min_arrival_time)
            ) {
            et_id = stop_time->trip_id;
//            std::cout << "Selected " << et_id << " d " << trips_[et_id].direction_id
//                      << " for " << std::setw(15) << stops_[pi_stop_id].stop_name
//                      << " (dep at " << stop_time->departure_time
//                      << " >= " << Utils::secondsToTime(min_arrival_time[pi_stop_id][k-1].min_arrival_time)
//                      << " min arr there [k-1]), target arr time: " << Utils::secondsToTime(min_arrival_time[query.target_id][k].min_arrival_time)
//                      << std::endl << std::endl;
            break; // We can break because stop_times is ordered
          }
        }

        if (et_id == "-1") continue; // No valid trip found for this stop

        Trip et = trips_[et_id];

//        std::cout << "Traversing remaining stops on route " << route_key.first << " direction " << route_key.second
//                  << " being et_id = " << et_id << std::endl;

        auto et_stop_it = std::find_if(et.stop_times.begin(), et.stop_times.end(),
                                    [&](StopTime* stoptime) { return stoptime->stop_id == pi_stop_id; });

        // Traverse remaining stops on the trip to update arrival times
        // TODO: is it okay to be the remaining stops on the trip? should I change back to 'remaining stops on the route'?
//        for (auto itt = std::next(it); itt != route.stops.end(); ++itt) {
        for (auto itt = std::next(et_stop_it); itt != et.stop_times.end(); ++itt) {

          StopTime* next_stop_time = *itt;
          std::string next_stop_id = next_stop_time->stop_id;

//          std::cout << " Next stop: " << next_stop_id << " " << next_stop->stop_name << std::endl;

          // Access arrival time at next_stop_id for trip et_id
          // TODO: why can't I use itt->arrival_time?
          int arrival_time = Utils::timeToSeconds(next_stop_time->arrival_time);

          // If arrival time can be improved, update Tk(pj) using et
          if (arrival_time < std::min(min_arrival_time[next_stop_id][k].min_arrival_time, min_arrival_time[query.target_id][k].min_arrival_time)) {
//            std::cout << "Marking " << std::setw(25) << stops_[next_stop_id].stop_name << et_id
//                      << " from " << std::setw(20) << stops_[pi_stop_id].stop_name
//                      << " "
//                      << Utils::secondsToTime(arrival_time) << " < min("
//                      << Utils::secondsToTime(min_arrival_time[next_stop_id][k].min_arrival_time)
//                      << ", "
//                      << Utils::secondsToTime(min_arrival_time[query.target_id][k].min_arrival_time)
//                      << ")" << std::endl;
            min_arrival_time[next_stop_id][k] = {arrival_time, et_id, pi_stop_id};
            marked_stops.insert(next_stop_id); // Mark this stop for the next round
          }

          // TODO: do it for current stop i or for subsequent stops j?
          // Check if an earlier trip can be caught at stop i (because a quicker path was found in a previous round)
          if ((min_arrival_time[next_stop_id][k-1].parent_trip_id != "-1") // Because we can instantly arrive at source
              && (min_arrival_time[next_stop_id][k - 1].min_arrival_time < arrival_time)) { // if Tk-1(pi) < Tarr(t, pi)

//            std::cout << "At stop " << std::setw(25) << stops_[next_stop_id].stop_name << " upt et_id " << et_id << " d" << trips_[et_id].direction_id
//                      << " -> " << min_arrival_time[next_stop_id][k-1].parent_trip_id
//                      << " d " << trips_[min_arrival_time[next_stop_id][k-1].parent_trip_id].direction_id
//                      << std::endl;

            // TODO: update t by recomputing et(r, pi) ---> would it mean only to update min_arrival_time array?
            //        does not make sense :')
            //       it indeed is the earliest trip that one can catch to get to next_stop_id,
            //       but this trip may not be caught from the original stop we were analysing
            //     so what do we do?
            //       et_id = min_arrival_time[next_stop_id][k-1].parent_trip_id;
            std::string parent_id =  min_arrival_time[next_stop_id][k-1].parent_trip_id;

//            std::cout << parent_id << "d" << trips_[parent_id].direction_id << "-" << std::setw(22) << stops_[next_stop_id].stop_name
//                      << " k-1 already there at " <<  Utils::secondsToTime(min_arrival_time[next_stop_id][k - 1].min_arrival_time)
//                      << " < " << Utils::secondsToTime(arrival_time) << " " << et_id << "d" << trips_[et_id].direction_id << " arr from " << pi_stop_id
//                      << " - break " // We can break, because we are analysing next_stop, we will check its earliest trip for each route
//                      << std::endl;
            break;
          }

        } // end remaining stops on route

      } // end each stop on route

      route_stop = routes_stops_set.erase(route_stop);
    } // end each route

    // Look for foot-paths
    // For each marked stop p
//    std::cout << std::endl;
    for (const auto& stop_id: marked_stops) {
      // For each foot-path (p, p')
      for (const auto& [dest_id, footpath]: stops_[stop_id].footpaths) {
        int new_arrival = min_arrival_time[stop_id][k].min_arrival_time + footpath.duration;
        // TODO: should I add this target comparison?
        if (new_arrival < std::min(min_arrival_time[dest_id][k].min_arrival_time, min_arrival_time[query.target_id][k].min_arrival_time)) {
//          std::cout << "Marking " << std::setw(22) << stops_[dest_id].stop_name
//                    << "fp from " << std::setw(22) << stops_[stop_id].stop_name
//                    << " " << Utils::secondsToTime(min_arrival_time[stop_id][k].min_arrival_time)
//                    << " + " << Utils::secondsToTime(footpath.duration)
//                    << " = " << Utils::secondsToTime(new_arrival)
//                    << " < min(" << Utils::secondsToTime(min_arrival_time[dest_id][k].min_arrival_time)
//                    << ", " << Utils::secondsToTime(min_arrival_time[query.target_id][k].min_arrival_time)
//                    << ")" << std::endl;
          min_arrival_time[dest_id][k].min_arrival_time = new_arrival;
          min_arrival_time[dest_id][k].parent_trip_id = "-1";
          min_arrival_time[dest_id][k].parent_stop_id = stop_id;
          marked_stops.insert(dest_id);
        }
      }
    }

    // Stopping criterion
    // if no stops are marked, then stop
    std::cout << "Marked " << std::setw(2) << marked_stops.size() << " stops during round " << k << std::endl;
//    for (const std::string &stop_id: marked_stops){
//      std::cout << stop_id << " " << std::setw(26) << stops_[stop_id].stop_name
//      << " " << Utils::secondsToTime(min_arrival_time[stop_id][k].min_arrival_time)
//      << " " << std::setw(6) << min_arrival_time[stop_id][k].parent_trip_id
//      << " " << stops_[min_arrival_time[stop_id][k].parent_stop_id].stop_name << std::endl;
//    }
//    std::cout << std::endl;

//    showMinArrivalTimes();
    k++;

    if (marked_stops.empty()) {
      break;
    }

  }


  std::vector<std::vector<JourneyStep>> journeys = reconstructJourneys(query);
  return journeys;
}


std::vector<std::vector<JourneyStep>> Raptor::reconstructJourneys(const Query &query) {
  std::vector<std::vector<JourneyStep>> journeys;

  std::cout << std::endl << "Reconstructing journeys..." << std::endl << std::endl ;

  // Start from the target stop and reconstruct the journey
  int current_k = k-1;
  while (current_k >= 0) {
    std::vector<JourneyStep> journey;
    std::string current_stop_id = query.target_id;
    bool found_journey = false;

    while (current_stop_id != "-1") {

      const std::string parent_trip_id = min_arrival_time[current_stop_id][current_k].parent_trip_id;
      const std::string parent_stop_id = min_arrival_time[current_stop_id][current_k].parent_stop_id;

      if (parent_stop_id == "-1") {
        found_journey = false;
        break;
      }

      int departure_time, arrival_time;
      if (parent_trip_id == "-1") { // Footpath
        int footpath_duration = stops_[parent_stop_id].footpaths[current_stop_id].duration;
        departure_time = min_arrival_time[parent_stop_id][current_k].min_arrival_time;
        arrival_time = departure_time + footpath_duration;
        break;
      } else { // Trip
        departure_time = Utils::timeToSeconds(stop_times_[{parent_trip_id, parent_stop_id}].departure_time);
        arrival_time = min_arrival_time[current_stop_id][current_k].min_arrival_time;
      }

      journey.push_back({parent_trip_id, parent_stop_id, current_stop_id, departure_time, arrival_time});

      // Update to the previous stop boarded
      current_stop_id = parent_stop_id;

      // Make sure we only add valid journeys
      if (current_stop_id == query.source_id) {
        found_journey = true;
        break;
      }
    }

    if (found_journey) {
      // Reverse the journey to obtain the correct sequence
      std::reverse(journey.begin(), journey.end());

      journeys.push_back(journey);
      current_k = (int) journey.size() - 1; // Decrease to the next journey with fewer transfers
    } else // No journey found
      break;

  }
  return journeys;
}

void Raptor::showMinArrivalTimes() {
  std::cout << std::endl << "    Minimal arrival times:" << std::endl << std::endl;

  // Print the header row
  std::cout << std::setw(6) << "Stop" << std::setw(28) << "Stop name";
  for (int current_k = 0; current_k <= k; current_k++)
    std::cout << std::setw(10) << current_k;

  std::cout << std::endl;

  // Print the arrival times for each stop
  for (const auto& [stop_id, arrivals] : min_arrival_time) {
    std::cout << std::setw(6) << stop_id;
    std::cout << std::setw(28) << stops_[stop_id].stop_name;

    for (const StopInfo& arrival : arrivals)
      std::cout << std::setw(10) << Utils::secondsToTime(arrival.min_arrival_time);

    std::cout << std::endl;
  }
}
