#include "application.h"
#include "bridge.h"
#include "d3d_window.h"
int main(){


#if !defined(DEBUG) && !defined(_DEBUG)
	FreeConsole();
#endif
	/* window title */
	d3d_window::_title = "bridge_demo";
	_application = new application();

	/* initialize application */
	if( _application->init() ){ 

		/* set application demo pointer */
		_application->m_demo = new bridge();

		/* initialize demo */
		if( _application->m_demo->init() ){

			/* start demo */
			_application->run(); 

			delete _application->m_demo;
		}
	}
}
