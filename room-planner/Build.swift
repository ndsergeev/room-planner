//
//  Build.swift
//  room-planner
//
//  Created by Nikita Sergeev on 19/6/2025.
//

struct BuildFlags {
    /// if walls are not included there possible issues related to non existence of another corner vertices
    static let includeWalls = true
    
    static let forceCloseWallLoop = true
}
