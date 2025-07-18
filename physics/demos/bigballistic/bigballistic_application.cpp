#include "application.h"
#include "bigballistic.h"
#include "d3d_window.h"

int main(){

#if !defined(DEBUG) && !defined(_DEBUG)
	FreeConsole();
#endif
	/* window title */
	d3d_window::_title = "bigballistic_demo";

	_application = new application();

	/* initialize application */
	if( _application->init() ){ 
		/* set application demo pointer */
		_application->m_demo = new bigballistic();

		/* initialize demo */
		if( _application->m_demo->init() ){

			/* start demo */
			_application->run(); 

			delete _application->m_demo;
		}
	}
}