/*
Copyright (C) 2018 Adolfo E. García

This file is part of STG-8nn-Scaffold.

STG-8nn-Scaffold is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

STG-8nn-Scaffold is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with STG-8nn-Scaffold.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "bsp.h"
#include "main.h"
#include "blinky.h"

/* -- Main loop -- */
int main(void)
{
  static QEvt const *blinky_queueSto[10]; /* Event queue buffer for Blinky */
  QF_init();
  BSP_Init();

  /* Instantiate and start the Blinky active object */
  Blinky_ctor();
  QACTIVE_START(AO_Blinky,  /* Active object to start */
    1U,                     /* Priority of the active object */
    blinky_queueSto,        /* Event queue buffer */
    Q_DIM(blinky_queueSto), /* Length of the buffer */
    (void *)0, 0U,          /* Private stack (not used) */
    (QEvt *)0);             /* Initialization event (not used) */

  return QF_run();
}