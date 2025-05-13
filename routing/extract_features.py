        
#feature_extractor.py

import math
import datetime

def extract_features(route, graph, world, driving_graph):
    total_distance = 0.0
    turns = 0
    junctions = 0
    total_delay = 0.0
    total_speed = 0.0
    segment_count = 0
    traffic_lights = 0
    
    for i in range (len(route) - 1):
        wp1 = graph.get_waypoint(route[i])
        wp2 = graph.get_waypoint(route[i + 1])
        
        dist = wp1.transform.location.distance(wp2.transform.location)
        total_distance += dist
        
        yaw1 = wp1.transform.rotation.yaw
        yaw2 = wp2.transform.rotation.yaw
        
        delta_yaw = abs(yaw2 - yaw1)
        if delta_yaw > 30:
            turns +=1
            
        if wp1.isjunction:
            junctions += 1
            
        total_speed += wp1.speed_limit * dist 
        
        seg_id_1 = (round(wp1.transform.location.x, 1), round(wp1.transform.location.y, 1))
        seg_id_2 = (round(wp2.transform.location.x, 1), round(wp2.transform.location.y, 1))
        seg = driving_graph.get(seg_id_1, {}).get(seg_id_2)
        
        if seg:
            abg_delay = seg["total_time"] / seg["samples"]
            total_delay += avg_delay
        else:
            total_delay += 2.5
            
        segment_count += 1
        
        avg_speed = total_speed / total_distance if total_distance else 0.0
        avg_delay = total_delay / segment_count if segment_count else 0.0
        
        hour = datetime.datetime.now().hour
        
        weather = world.get_weather()
        weather_code = _weather_to_code(weather)
        
        lights = world.get_actors().filter("traffic.traffic_light")
        for wp in [graph.get_waypoint(n) for n in route]:
            for light in lights:
                if wp.transform.location.distance(light.get_transform().location) < 10.0:
                    traffic_lights += 1
                    break
                
        return [
            total_distance,
            turns,
            junctions,
            avg_delay,
            avg_speed,
            hour,
            weather_code,
            traffic_lights
        ]
        
        
def _weather_to_code(weather):
    
        if weather.percipitation > 50:
            return 2 #heavy rain
        elif weather.cloudiness > 50:
            return 1
        else:
            return 0
        