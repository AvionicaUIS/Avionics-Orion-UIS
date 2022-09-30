/*
 * Archivo de cabecera o header, por eso la extensión .h
 * Aquí se instancian las funciones, es decir, aquí se define la sintaxis que llevarán y la lógica o procedimiento se almacena en archivos .cpp
 * Ejemplo, creo la lógica la cual irá en el .cpp, aquí defino el tipo de valor de retorno, los parámetros de la función y demás
 * En el archivo principal, únicamente llamaré la función aquí definida.
 */

#ifndef PRUEBALIBRERIA_H
#define PRUEBALIBRERIA_H

#include <Arduino.h>

int addTwoInts(int a, int b);

#endif
