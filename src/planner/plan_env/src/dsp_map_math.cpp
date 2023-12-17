#include "plan_env/dsp_map_math.hpp"

#include <cmath>
#include <limits>
#include <iostream>

using std::numeric_limits;



inline Vector floor(const Vector& index_vector)
{
    Vector res;
    for(int i=0;i<index_vector.size();i++)
    {
        res[i] = std::floor(index_vector[i]);
    }
    return res;
}
inline Vector ceil(const Vector& index_vector)
{
    Vector res;
    for(int i=0;i<index_vector.size();i++)
    {
        res[i] = std::ceil(index_vector[i]);
    }
    return res;
}

inline bool getVectorToOrigin(Vector& vector_to_origin, const Length& map_length)
{
    vector_to_origin = (-0.5 * map_length).matrix();
    return true;
}

inline bool getVectorToFirstCell(Vector& vector_to_first_cell, const Length& map_length,const float& resolution)
{
    Vector vector_to_origin;
    getVectorToOrigin(vector_to_origin,map_length);
    vector_to_first_cell = (vector_to_origin.array() + 0.5 * resolution).matrix();
    return true;
}  

inline bool checkIfStartIndexAtDefaultPosition(const Index& buffer_start_index)
{
    return ((buffer_start_index.array() == 0).all());
}

inline Index getIndexFromIndexVector(const Vector& index_vector, const Size& buffer_size, const Index& buffer_start_index)
{
    
    Index index = floor(index_vector).cast<int>();
    return getBufferIndexFromIndex(index,buffer_size,buffer_start_index);
}

/// @brief 把循环缓存里的index转换为IndexVector
/// @param index 
/// @param buffer_size 
/// @param buffer_start_index 
/// @return 
inline Vector getIndexVectorFromIndex(
    const Index& index,
    const Size& buffer_size,
    const Index& buffer_start_index)
{
    Index unwrapped_index;
    unwrapped_index = getIndexFromBufferIndex(index,buffer_size,buffer_start_index);
    return unwrapped_index.cast<float>();
}




bool getPositionFromIndex(  Position &position, 
                            const Index &index, 
                            const Length &map_length, 
                            const Position &map_position, 
                            const float &resolution, 
                            const Size &buffer_size, 
                            const Index &buffer_start_index)
{
    if(!checkIfIndexInRange(index,buffer_size)){
        return false;
    }
    Vector offset;
    getVectorToFirstCell(offset,map_length,resolution);
    position = map_position + offset + resolution * getIndexVectorFromIndex(index,buffer_size,buffer_start_index);
    return true;
}

bool getIndexFromPosition(  Index& index,
                            const Position& position,
                            const Length& map_length,
                            const Position& map_position,
                            const float& resolution,
                            const Size& buffer_size,
                            const Index& buffer_start_index)
{
    Vector offset;
    getVectorToOrigin(offset,map_length);
    Vector index_vector = ((position - offset - map_position).array() / resolution).matrix();
    // printf("position: %f\t%f\t%f\n",position[0],position[1],position[2]);
    // printf("offset: %f\t%f\t%f\n",offset[0],offset[1],offset[2]);
    // printf("map_position: %f\t%f\t%f\n",map_position[0],map_position[1],map_position[2]);
    // printf("index_vector: %f\t%f\t%f\n",index_vector[0],index_vector[1],index_vector[2]);
    index = getIndexFromIndexVector(index_vector,buffer_size,buffer_start_index);
    // printf("index: %d\t%d\t%d\n",index[0],index[1],index[2]);

    return checkIfPositionWithinMap(position,map_length,map_position) && checkIfIndexInRange(index,buffer_size);
}


bool checkIfPositionWithinMap(  const Position& position,
                                const Length& map_length,
                                const Position& map_position)
{
    Vector offset;
    getVectorToOrigin(offset,map_length);
    Position position_transformed = (position - map_position - offset);
    // printf("offset: %f\t%f\t%f\n",offset[0],offset[1],offset[2]);
    // printf("map_position: %f\t%f\t%f\n",map_position[0],map_position[1],map_position[2]);
    // printf("position_transformed: %f\t%f\t%f\n",position_transformed[0],position_transformed[1],position_transformed[2]);
    // printf("map_length: %f\t%f\t%f\n",map_length[0],map_length[1],map_length[2]);

    return position_transformed.x() >= 0.0 && position_transformed.y() >= 0.0 && position_transformed.z() >= 0.0 
        && position_transformed.x() < map_length(0) && position_transformed.y() < map_length(1) && position_transformed.z() < map_length(2);
}


void getPositionOfDataStructureOrigin(  const Position& position,
                                        const Length& map_length,
                                        Position& position_of_origin)
{
    Vector vector_to_origin;
    getVectorToOrigin(vector_to_origin,map_length);
    position_of_origin = position + vector_to_origin;
}

bool getIndexShiftFromPositionShift(Index& index_shift,
                                    const Vector& position_shift,
                                    const float& resolution)
{


    Vector index_shift_vector_temp = (position_shift.array() / resolution).matrix();
    Vector index_shift_vector;
    for(int i=0;i<index_shift_vector.size();i++){
        index_shift_vector[i] = index_shift_vector_temp[i] + 0.5 * (index_shift_vector_temp[i] > 0 ? 1 : -1);
    }
    index_shift = index_shift_vector.cast<int>();
    return true;
}


bool getPositionShiftFromIndexShift(Vector& position_shift,
                                    const Index& index_shift,
                                    const float& resolution)
{
    position_shift = index_shift.cast<float>() * resolution;
    return true;
}

bool checkIfIndexInRange(const Index& index,const Size& buffer_size)
{
    // printf("index: %d\t%d\t%d\n",index[0],index[1],index[2]);
    // printf("buffer_size: %d\t%d\t%d\n",buffer_size[0],buffer_size[1],buffer_size[2]);
    return index[0] >= 0 && index[1] >= 0 && index[2] >= 0
            && index[0] < buffer_size[0] && index[1] < buffer_size[1] && index[2] < buffer_size[2];
}


void boundIndexToRange(Index& index, const Size& buffer_size)
{
    for (int i = 0; i < index.size(); i++) {
        boundIndexToRange(index[i], buffer_size[i]);
    }
}

void boundIndexToRange(int& index, const int& buffer_size)
{
    if (index < 0) {
        index = 0;
    } else if (index >= buffer_size) {
        index = buffer_size - 1;
    }
}

void wrapIndexToRange(Index& index, const Size& buffer_size)
{
    for (int i = 0; i < index.size(); i++) {
        wrapIndexToRange(index[i], buffer_size[i]);
    }
}

void wrapIndexToRange(int& index, int buffer_size)
{
    // Try shortcuts before resorting to the expensive modulo operation.
    if (index < buffer_size){
        if(index >= 0){ // within the wanted range
            return;
        } else if(index >= -buffer_size){ // Index is below range, but not more than one span of the range.
            index +=buffer_size;
            return;
        }else{ // Index is largely below range.
            index = index % buffer_size;
            index += buffer_size;
        }
    }else if(index < buffer_size*2){ // Index is above range, but not more than one span of the range.
        index -= buffer_size;
        return;
    } else{ // Index is largely above range.
        index = index % buffer_size;
    }
}

void boundPositionToRange(Position& position, const Length& map_length, const Position& map_position)
{
    Vector vector_to_origin;
    getVectorToOrigin(vector_to_origin, map_length); // 指向地图原点的向量
    Position position_shifted = position - map_position + vector_to_origin; // (1) position shift (2) map左上角->map中心的距离 = 点在地图中的相对位置

    // We have to make sure to stay inside the map.
    for (int i = 0; i < position_shifted.size(); i++) {

        double epsilon = 10.0 * numeric_limits<double>::epsilon(); // TODO Why is the factor 10 necessary.
        if (std::fabs(position(i)) > 1.0) {
            epsilon *= std::fabs(position(i));
        }

        if (position_shifted(i) <= 0) { 
            position_shifted(i) = epsilon; 
            continue;
        }
        if (position_shifted(i) >= map_length(i)) {
            position_shifted(i) = map_length(i) - epsilon;
            continue;
        }
    }
    // 把超出地图的position shift bound 到 [epsilon, mapLength - epislon] 中
    position = position_shifted + map_position - vector_to_origin; // line 3 的逆操作，此时position_shift 一定在地图内了，因此，取出点的绝对位置。
}

Index getBufferIndexFromIndex(const Index& index, const Size& buffer_size, const Index& buffer_start_index)
{
    if(checkIfStartIndexAtDefaultPosition(buffer_start_index)){
        return index;
    }
    Index buffer_index = index + buffer_start_index;
    wrapIndexToRange(buffer_index,buffer_size);
    return buffer_index;
}

Index getIndexFromBufferIndex(const Index& buffer_index,const Size& buffer_size,const Index& buffer_start_index)
{
    if(checkIfStartIndexAtDefaultPosition(buffer_start_index)){
        return buffer_index;
    }
    Index index = buffer_index - buffer_start_index;
    wrapIndexToRange(index,buffer_size);
    return index;
}

int getAddressFromBufferIndex(const Index& index,const Size& bufferSize)
{
    return index(0) * bufferSize(1) * bufferSize(2) + index(1) * bufferSize(2) + index(2);
}


// void getPositionFromAddress(Position& position, const int& address,const Size& bufferSize)
// {
//     int 
// }


