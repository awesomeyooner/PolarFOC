#include "PolarFOC/filters/filter.hpp"


double Filter::get(double incoming_data)
{
    update(incoming_data);

    return get();

} // end of "get(double incoming_data)"