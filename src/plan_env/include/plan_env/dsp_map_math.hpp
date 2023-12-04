#pragma once

#include "plan_env/Typdefs.hpp"
#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <map>



namespace dsp_map
{

inline Vector floor(Vector& vector);

// Index transformMapFrameToBufferOrder(const Vector& index_vector);
// Index transformMapFrameToBufferOrder(const Vector& index_vector); // index_vector -> index
// Vector transformBufferOrderToMapFrame(const Index& index); // index -> index_vector
Vector floor(const Vector& vector);

// Index transformMapFrameToBufferOrder(const Index& index);
// Vector transformMapFrameToBufferOrder(const Vector& index_vector);
// Index transformBufferOrderToMapFrame(const Index& index);
// Vector transformBufferOrderToMapFrame(const Vector& index_vector);
bool getVectorToOrigin(Vector& vector_to_origin, const Length& map_length);
bool getVectorToFirstCell(Vector& vector_to_first_cell, const Length& map_length,const float& resolution);

bool checkIfStartIndexAtDefaultPosition(const Index& buffer_start_index);
Index getIndexFromIndexVector(const Vector& index_vector, const Size& buffer_size, const Index& buffer_start_index);
Vector getIndexVectorFromIndex(
    const Index& index,
    const Size& buffer_size,
    const Index& buffer_start_index);

/*!
 * Gets the position of a cell specified by its index in the map frame.
 * @param[out] position the position of the center of the cell in the map frame.
 * @param[in] index of the cell.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer (optional).
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if index not within range of buffer.
 */
bool getPositionFromIndex(Position& position,
                          const Index& index,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const float& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex = Index::Zero());

/*!
 * Gets the index of the cell which contains a position in the map frame.
 * @param[out] index of the cell.
 * @param[in] position the position in the map frame.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer (optional).
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if position outside of map.
 */
bool getIndexFromPosition(Index& index,
                          const Position& position,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const float& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex = Index::Zero());


/*!
 * Checks if position is within the map boundaries.
 * @param[in] position the position which is to be checked.
 * @param[in] mapLength the length of the map.
 * @param[in] mapPosition the position of the map.
 * @return true if position is within map, false otherwise.
 */
bool checkIfPositionWithinMap(const Position& position,
                              const Length& mapLength,
                              const Position& mapPosition);

/*!
 * Computes how many cells/indices the map is moved based on a position shift in
 * the grid map frame. Use this function if you are moving the grid map
 * and want to ensure that the cells match before and after.
 * @param[out] indexShift the corresponding shift of the indices.
 * @param[in] positionShift the desired position shift.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
bool getIndexShiftFromPositionShift(Index& indexShift,
                                    const Vector& positionShift,
                                    const float& resolution);


/*!
 * Computes the corresponding position shift from a index shift. Use this function
 * if you are moving the grid map and want to ensure that the cells match
 * before and after.
 * @param[out] positionShift the corresponding shift in position in the grid map frame.
 * @param[in] indexShift the desired shift of the indices.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
bool getPositionShiftFromIndexShift(Vector& positionShift,
                                    const Index& indexShift,
                                    const float& resolution);


/*!
 * Checks if index is within range of the buffer.
 * @param[in] index to check.
 * @param[in] bufferSize the size of the buffer.
 * @return true if index is within, and false if index is outside of the buffer.
 */
bool checkIfIndexInRange(const Index& index, const Size& bufferSize);                                    


/*!
 * Bounds an index that runs out of the range of the buffer.
 * This means that an index that overflows is stopped at the last valid index.
 * This is the 2d version of boundIndexToRange(int&, const int&).
 * @param[in/out] index the indices that will be bounded to the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void boundIndexToRange(Index& index, const Size& bufferSize);


/*!
 * Bounds an index that runs out of the range of the buffer.
 * This means that an index that overflows is stopped at the last valid index.
 * @param[in/out] index the index that will be bounded to the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void boundIndexToRange(int& index, const int& bufferSize);


/*!
 * Wraps an index that runs out of the range of the buffer back into allowed the region.
 * This means that an index that overflows is reset to zero.
 * This is the 2d version of wrapIndexToRange(int&, const int&).
 * @param[in/out] index the indices that will be wrapped into the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void wrapIndexToRange(Index& index, const Size& bufferSize);


/*!
 * Wraps an index that runs out of the range of the buffer back into allowed the region.
 * This means that an index that overflows is reset to zero.
 * @param[in/out] index the index that will be wrapped into the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void wrapIndexToRange(int& index, int bufferSize);


/*!
 * Bound (cuts off) the position to lie inside the map.
 * This means that an index that overflows is stopped at the last valid index.
 * @param[in/out] position the position to be bounded.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 */
void boundPositionToRange(Position& position, const Length& mapLength, const Position& mapPosition);



/*!
 * Retrieve the index as unwrapped index, i.e., as the corresponding index of a
 * grid map with no circular buffer offset.
 * @param bufferIndex the index in the circular buffer.
 * @param bufferSize the map buffer size.
 * @param bufferStartIndex the map buffer start index.
 * @return the unwrapped index.
 */
Index getIndexFromBufferIndex(const Index& bufferIndex, const Size& bufferSize,
                              const Index& bufferStartIndex);


/*!
 * Retrieve the index of the buffer from a unwrapped index (reverse from function above).
 * @param index the unwrapped index.
 * @param bufferSize the map buffer size.
 * @param bufferStartIndex the map buffer start index.
 * @return the buffer index.
 */
Index getBufferIndexFromIndex(const Index& index, const Size& bufferSize, const Index& bufferStartIndex);

/*!
 * Get Memory address from the local buffer index
 * @param index buffer index
 * @param bufferSize the map buffer size.
 * @return the memory address.
 */
int getAddressFromBufferIndex(const Index& index,const Size& bufferSize);

/*!
 * Get voxel position from buffer address
 * @param position the output position of the voxel
 * @param address the input buffer address
 * @param bufferSize the map buffer size.
 * @return the memory address.
 */
void getPositionFromAddress(Position& position, const int& address,const Size& bufferSize);



}

