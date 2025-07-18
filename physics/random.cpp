#include "random.h"

#include <cstdlib>
#include <ctime>

random::random() {
    seed(0);
}

random::random(uint32_t seed){
    random::seed(seed);
}

void random::seed(uint32_t s){

    if (s == 0) { s = (uint32_t)clock(); }

    // fill the buffer with some basic random numbers
    for (uint32_t i = 0; i < 17; i++) {
        // simple linear congruential generator
        s = s * 2891336453 + 1;
        m_buffer[i] = s;
    }

    // initialize pointers into the buffer
    m_p1 = 0;  m_p2 = 10;
}

uint32_t random::rotl(uint32_t n, uint32_t r){
	  return	(n << r) |  (n >> (32 - r));
}

uint32_t random::rotr(uint32_t n, uint32_t r){
	  return	(n >> r) | (n << (32 - r));
}

uint32_t random::randombits() {
    uint32_t result;

    // rotate the buffer and store it back to itself
    result = m_buffer[m_p1] = rotl(m_buffer[m_p2], 13) + rotl(m_buffer[m_p1], 9);

    // rotate pointers
	if (--m_p1 < 0) { m_p1 = 16; }
	if (--m_p2 < 0) { m_p2 = 16; }

    // return result
    return result;
}

float random::randomreal() {

    // get the random number
    uint32_t bits = randombits();

    // set up a reinterpret structure for manipulation
    union {
        float value;
        uint32_t word;
    } convert;

    // now assign the bits to the word. this works by fixing the ieee
    // sign and exponent bits (so that the size of the result is 1-2)
    // and using the bits to create the fraction part of the float.
    convert.word = (bits >> 9) | 0x3f800000;

    // and return the value
    return convert.value - 1.0f;
}


float random::randomreal(float min, float max) { return randomreal() * (max-min) + min; }

float random::randomreal(float scale)          { return randomreal() * scale; }

uint32_t random::randomint(uint32_t max)       { return randombits() % max; }

float random::randombinomial(float scale)      { return (randomreal()-randomreal())*scale; }

_quaternion random::randomquaternion() {
    _quaternion q(
        randomreal(),
        randomreal(),
        randomreal(),
        randomreal()
        );
    q.normalise();
    return q;
}

_vec3 random::randomvector(float scale) {
    return _vec3( randombinomial(scale), randombinomial(scale), randombinomial(scale) );
}

_vec3 random::randomxzvector(float scale){
    return _vec3(
        randombinomial(scale),
        0,
        randombinomial(scale)
        );
}

_vec3 random::randomvector(const _vec3 &scale) {
    return _vec3(
        randombinomial(scale.x),
        randombinomial(scale.y),
        randombinomial(scale.z)
        );
}

_vec3 random::randomvector(const _vec3 &min, const _vec3 &max) {
    return _vec3( randomreal(min.x, max.x), randomreal(min.y, max.y), randomreal(min.z, max.z) );
}
