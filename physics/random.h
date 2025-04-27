
#include "application_header.h"


/**
* keeps track of one random stream: i.e. a seed and its output.
* this is used to get random numbers. rather than a funcion, this
* allows there to be several streams of repeatable random numbers
* at the same time. uses the randrotb algorithm.
*/
class random {
public:

	/**
	* left bitwise rotation
	*/
	uint32_t rotl(uint32_t n, uint32_t r);
	/**
	* right bitwise rotation
	*/
	uint32_t rotr(uint32_t n, uint32_t r);

	/**
	* creates a new random number stream with a seed based on
	* timing data.
	*/
	random();

	/**
	* creates a new random stream with the given seed.
	*/
	random(uint32_t seed);

	/**
	* sets the seed value for the random stream.
	*/
	void seed(uint32_t seed);

	/**
	* returns the next random bitstring from the stream. this is
	* the fastest method.
	*/
	uint32_t randombits();

	/**
	* returns a random floating point number between 0 and 1.
	*/
	float randomreal();

	/**
	* returns a random floating point number between 0 and scale.
	*/
	float randomreal(float scale);

	/**
	* returns a random floating point number between min and max.
	*/
	float randomreal(float min, float max);

	/**
	* returns a random integer less than the given value.
	*/
	uint32_t randomint(uint32_t max);

	/**
	* returns a random binomially distributed number between -scale
	* and +scale.
	*/
	float randombinomial(float scale);

	/**
	* returns a random vector where each component is binomially
	* distributed in the range (-scale to scale) [mean = 0.0f].
	*/
	_vec3 randomvector(float scale);

	/**
	* returns a random vector where each component is binomially
	* distributed in the range (-scale to scale) [mean = 0.0f],
	* where scale is the corresponding component of the given
	* vector.
	*/
	_vec3 randomvector(const _vec3 &scale);

	/**
	* returns a random vector in the cube defined by the given
	* minimum and maximum vectors. the probability is uniformly
	* distributed in this region.
	*/
	_vec3 randomvector(const _vec3 &min, const _vec3 &max);

	/**
	* returns a random vector where each component is binomially
	* distributed in the range (-scale to scale) [mean = 0.0f],
	* except the y coordinate which is zero.
	*/
	_vec3 randomxzvector(float scale);

	/**
	* returns a random orientation (i.e. normalized) quaternion.
	*/
	_quaternion randomquaternion();

private:
	// internal mechanics
	int32_t  m_p1;
	int32_t  m_p2;
	uint32_t m_buffer[17];
};