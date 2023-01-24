/*******************************************************************************
 * Copyright (c) 2016-2019 Embedded Systems and Applications Group
 * Department of Computer Science, Technische Universitaet Darmstadt,
 * Hochschulstr. 10, 64289 Darmstadt, Germany.
 *
 * All rights reserved.
 *
 * This software is provided free for educational use only.
 * It may not be used for commercial purposes without the
 * prior written permission of the authors.
 ******************************************************************************/
package mtam;

public class DisassemblyLine {
	public final Type type;
	public final int address;
	public final Instruction inst;
	public final String text;
	
	public DisassemblyLine(int address) {
		this.type = Type.empty;
		this.address = address;
		this.inst = null;
		this.text = null;
	}
	
	public DisassemblyLine(int address, Instruction inst) {
		this.type = Type.instruction;
		this.address = address;
		this.inst = inst;
		this.text = null;
	}
	
	public DisassemblyLine(int address, String text, Type type) {
		this.type = type;
		this.address = address;
		this.inst = null;
		this.text = text;
	}
	
	public enum Type {
		empty, comment, label, instruction
	}
}
