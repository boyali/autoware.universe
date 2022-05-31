//
// Created by ali on 24/05/22.
//

#include "integration/integrate_eigen_states.hpp"

int main()
{

	auto dummyode_1 = dummyOde(1);
	auto dummyode_2 = dummyOde(2);

	// Dummy Type Erasure for operator () interface.
	ODE_zoh_common interface{ dummyode_1 };
	interface.print();
	interface();

	// Now change.
	interface = dummyode_2;
	interface.print();
	interface();

	// interface();

	return 0;
}