

#include "log_stats.h"
#include <cmath>


float Average(const std::vector<float>& data)
{
    double average = 0.0;
    const int size = data.size();
    const double size_scalar = 1.0/size;
    for(int i = 0; i < size; i++)
    {
        // dont factor out multiply to improve float accuracy
        average += data[i]*size_scalar;
    }
    return average;
}

float StandardDeviation(const std::vector<float>& data)
{
    return StandardDeviation(data, Average(data));
}

float StandardDeviation(const std::vector<float>& data, const float average)
{
    double standard_deviation = 0.0;
    const int size = data.size();
    const double size_scalar = 1.0/size;
    const double averaged = average;
    for(int i = 0; i < size; i++)
    {
        // dont factor out multiply to improve float accuracy
        const double difference = data[i] - averaged;
        standard_deviation += difference*difference*size_scalar;
    }
    return standard_deviation;
}
        

float TotalMouseDistance(const LOG_DATA& log_data)
{
   const int size = log_data.frames.size();
   float distance = 0.0f;
   float previous_x = log_data.frames[0].mouse.x;
   float previous_y = log_data.frames[0].mouse.y;
   for(int i = 1; i < size; i++)
   {
      const float current_x = log_data.frames[i].mouse.x;
      const float current_y = log_data.frames[i].mouse.y;
      const float x = (current_x - previous_x);
      const float y = (current_y - previous_y);
      distance += sqrt(x*x + y*y);
      
      previous_x = current_x;
      previous_y = current_y;
   }
   return distance;
}

float TotalShepherdDistance(const LOG_DATA& log_data, const int which_shepherd)
{
   const int size = log_data.frames.size();
   float distance = 0.0f;
   float previous_x = log_data.frames[0].shepherds[which_shepherd].x;
   float previous_y = log_data.frames[0].shepherds[which_shepherd].y;
   for(int i = 1; i < size; i++)
   {
      const float current_x = log_data.frames[i].shepherds[which_shepherd].x;
      const float current_y = log_data.frames[i].shepherds[which_shepherd].y;
      const float x = (current_x - previous_x);
      const float y = (current_y - previous_y);
      distance += sqrt(x*x + y*y);
      
      previous_x = current_x;
      previous_y = current_y;
   }
   return distance;
}

std::vector<float> TotalShepherdDistance(const LOG_DATA& log_data)
{
   std::vector<float> distances(log_data.num_shepherds);
   const int size = distances.size();
   for(int i = 0; i < size; i++)
   {
      distances[i] = TotalShepherdDistance(log_data, i);
   }
   return distances;
}

float AverageSheepSeparation(const LOG_DATA& log_data)
{
    double total_separation = 0.0f;
    const int num_frames = log_data.frames.size();
    const int num_sheep = log_data.num_sheep;
    double sheep_scalar = 2.0/(num_sheep*(num_sheep - 1.0));
    for(int i = 0; i < num_frames; i++)
    {
        double distance = 0.0f;
        for(int j = 0; j < num_sheep; j++)
        {
            for(int k = j + 1; k < num_sheep; k++)
            {
                distance += log_data.frames[i].sheep[j].Distance(log_data.frames[i].sheep[k]);
            }
        }
        total_separation += distance*sheep_scalar;
    }
    return (total_separation/(double)num_frames);
}

float AverageNumberOfGroups(const LOG_DATA& log_data, const float distance_threshold)
{
    double average_groups = 0.0f;
    const int num_frames = log_data.frames.size();
    const double frame_scalar = 1.0/num_frames;
    for(int i = 0; i < num_frames; i++)
    {
        // copy the sheep vector into a new pool
        std::vector<POINT2F> remaining_sheep = log_data.frames[i].sheep;
        int num_groups = 0;
        
        // while there are still sheep left without a group...
        while(remaining_sheep.size() > 0)
        {
            const int num_remaining = remaining_sheep.size();
            // initialize an empty list of group members for each sheep
            std::vector<std::vector<int> > candidate_groups(num_remaining);
            
            // for each sheep..
            for(int j = 0; j < num_remaining; j++)
            {
                // TODO save distance calculations for reuse
                for(int k = 0; k < num_remaining; k++)
                {
                    // add each sheep in range to its group
                    if(log_data.frames[i].sheep[j].Distance(log_data.frames[i].sheep[k]) <= distance_threshold)
                    {
                        candidate_groups[j].push_back(k);
                    }
                }
            }
            // find the largest of the groups
            int biggest_group = 0;
            for(int j = 1; j < candidate_groups.size(); j++)
            {
                if(candidate_groups[j].size() > candidate_groups[biggest_group].size())
                {
                    biggest_group = j;
                }
            }
            // remove that group's sheep from the ungrouped pool (in reverse order)
            for(int j = candidate_groups[biggest_group].size() - 1; j >= 0; j--)
            {
                remaining_sheep.erase(remaining_sheep.begin() + candidate_groups[biggest_group][j]);
            }
            num_groups++;
        }
        average_groups += ((double)num_groups)*frame_scalar;
    }
    return average_groups;
}
    
        
            
    
