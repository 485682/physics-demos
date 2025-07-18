#include "application.h"
#include "explosion.h"
#include "d3d_window.h"
#include "body.h"

int main(){

#if !defined(DEBUG) && !defined(_DEBUG)
	FreeConsole();
#endif
	/* window title */
	d3d_window::_title = "explosion_demo";

	_application = new application();

	/* initialize application */
	if( _application->init() ){ 
		/* set application demo pointer */
		_application->m_demo = new explosion();

		/* initialize demo */
		if( _application->m_demo->init() ){

			/* start demo */
			_application->run(); 

			delete _application->m_demo;
		}
	}
}