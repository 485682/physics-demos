#include "application.h"
#include "ballistic.h"
#include "d3d_window.h"

int main(){


#if !defined(DEBUG) && !defined(_DEBUG)
	FreeConsole();
#endif

	/* window title */
	d3d_window::_title = "ballistic demo ";

	_application = new application();
	/* initialize application */
	if( _application->init() ){ 

		/* set application demo pointer */
		_application->m_demo = new ballistic();
		/* initialize demo */
		if( _application->m_demo->init() ){

			/* start demo */
			_application->run(); 

			delete _application->m_demo;
		}

	}
}