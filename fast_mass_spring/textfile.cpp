#include "textfile.h"

Textfile::Textfile(const char* title) {
	txt.open(title);
}

Textfile::~Textfile() {
	Close();
}
void Textfile::Close() {
	valid = false;
	txt.close();
}