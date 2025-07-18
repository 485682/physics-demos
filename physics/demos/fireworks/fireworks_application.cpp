#include "application.h"
#include "fireworks.h"
#include "d3d_window.h"

int main(){


#if !defined(DEBUG) && !defined(_DEBUG)
	FreeConsole();
#endif
	/* window title */
	d3d_window::_title = "fireworks_demo";

	_application = new application();
	/* initialize application */
	if( _application->init() ){ 

		/* set application demo pointer */
		_application->m_demo = new fireworks();

		/* initialize demo */
		if( _application->m_demo->init() ){

			/* set d3d clear color to black (night) */
			application::_clear_color = D3DCOLOR_XRGB(0, 0,0);

			/* start demo */
			_application->run(); 

			delete _application->m_demo;
		}
	}
}