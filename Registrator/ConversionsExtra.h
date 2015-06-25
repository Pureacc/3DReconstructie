#ifndef __CONVERSIONSEXTRA_H_INCLUDED__
#define __CONVERSIONSEXTRA_H_INCLUDED__

namespace conversionsExtra {

	//Remember to delete returned pointer afterwards
	wchar_t* toWchar(const char* str) {
		size_t strsize = strlen(str) + 1;
		wchar_t* strnew = new wchar_t[strsize];
		size_t convertedChars = 0;
		mbstowcs_s(&convertedChars, strnew, strsize, str, _TRUNCATE);
		return strnew;
	}

}

#endif
