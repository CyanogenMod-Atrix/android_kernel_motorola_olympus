/*
 * Copyright 2013: Olympus Kernel Project
 * <http://forum.xda-developers.com/showthread.php?t=2016837>
 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

static char tomoyo_builtin_profile[] __initdata =
"";
static char tomoyo_builtin_exception_policy[] __initdata =
"initialize_domain /sbin/modprobe from any\n"
"initialize_domain /sbin/hotplug from any\n"
"";
static char tomoyo_builtin_domain_policy[] __initdata =
"";
static char tomoyo_builtin_manager[] __initdata =
"";
static char tomoyo_builtin_stat[] __initdata =
"";
