#!/bin/bash

# Check if the minimum number of arguments is provided
if [ "$#" -lt 3 ]; then
    echo "Usage: $0 <input_xacro_file> <scale_factor> <frame_prefix> [xacro_args...]"
    exit 1
fi

INPUT_XACRO=$1
SCALE_FACTOR=$2
FRAME_PREFIX=$3
shift 3 # Shift the arguments so that $@ contains only the xacro arguments

# Check if xacro is installed
if ! command -v xacro &>/dev/null; then
    echo "Error: xacro is not installed. Please install it using:"
    echo "sudo apt-get install ros-<rosdistro>-xacro"
    exit 1
fi

# Process the input XACRO with xacro, passing any additional arguments
TEMP_URDF=$(mktemp)
xacro "$INPUT_XACRO" frame_prefix:="$FRAME_PREFIX" "$@" >"$TEMP_URDF"
if [ $? -ne 0 ]; then
    rm "$TEMP_URDF"
    exit 1
fi

# Create an XSLT stylesheet to scale the URDF and apply frame prefix
XSLT_STYLESHEET=$(mktemp)

cat <<EOF >$XSLT_STYLESHEET
<?xml version="1.0"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

    <!-- Define the parameters -->
    <xsl:param name="scale_factor"/>
    <xsl:param name="frame_prefix"/>

    <!-- Identity transform for all elements by default -->
    <xsl:template match="@*|node()">
        <xsl:copy>
            <xsl:apply-templates select="@*|node()"/>
        </xsl:copy>
    </xsl:template>

    <!-- Prefix link/name, joint/name, joint/parent/link, joint/child/link, and gazebo/reference -->
    <xsl:template match="link/@name | joint/@name | joint/parent/@link | joint/child/@link | gazebo/@reference">
        <xsl:variable name="name" select="."/>
        <xsl:variable name="attr" select="name()"/>
        <xsl:attribute name="{\$attr}">
            <xsl:value-of select="concat(\$frame_prefix, \$name)"/>
        </xsl:attribute>
    </xsl:template>

    <!-- Prefix joint names used inside Gazebo plugin tags -->
    <xsl:template match="left_joint | right_joint | left_steering_joint | right_steering_joint | joint_name">
        <xsl:copy>
            <xsl:value-of select="concat(\$frame_prefix, .)"/>
        </xsl:copy>
    </xsl:template>

    <!-- Scale the origin xyz attribute -->
    <xsl:template match="origin/@xyz">
        <xsl:attribute name="xyz">
            <xsl:call-template name="scale-space-separated">
                <xsl:with-param name="value" select="."/>
            </xsl:call-template>
        </xsl:attribute>
    </xsl:template>

    <!-- Scale the size attribute in geometry/box -->
    <xsl:template match="geometry/box/@size">
        <xsl:attribute name="size">
            <xsl:call-template name="scale-space-separated">
                <xsl:with-param name="value" select="."/>
            </xsl:call-template>
        </xsl:attribute>
    </xsl:template>

    <!-- Scale the radius and length attributes in geometry/cylinder -->
    <xsl:template match="geometry/cylinder/@radius | geometry/cylinder/@length">
        <xsl:attribute name="{name()}">
            <xsl:value-of select="number(.) * number(\$scale_factor)"/>
        </xsl:attribute>
    </xsl:template>

    <!-- Scale the radius attribute in geometry/sphere -->
    <xsl:template match="geometry/sphere/@radius">
        <xsl:attribute name="radius">
            <xsl:value-of select="number(.) * number(\$scale_factor)"/>
        </xsl:attribute>
    </xsl:template>

    <!-- Scale the mesh scale attribute -->
    <xsl:template match="geometry/mesh/@scale">
        <xsl:attribute name="scale">
            <xsl:call-template name="scale-space-separated">
                <xsl:with-param name="value" select="."/>
            </xsl:call-template>
        </xsl:attribute>
    </xsl:template>

    <!-- Template to scale space-separated values -->
    <xsl:template name="scale-space-separated">
        <xsl:param name="value"/>
        <xsl:param name="delimiter" select="' '"/>

        <xsl:choose>
            <xsl:when test="contains(\$value, \$delimiter)">
                <xsl:value-of select="number(substring-before(\$value, \$delimiter)) * number(\$scale_factor)"/>
                <xsl:text> </xsl:text>
                <xsl:call-template name="scale-space-separated">
                    <xsl:with-param name="value" select="substring-after(\$value, \$delimiter)"/>
                </xsl:call-template>
            </xsl:when>
            <xsl:otherwise>
                <xsl:value-of select="number(\$value) * number(\$scale_factor)"/>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:template>

</xsl:stylesheet>
EOF

# Check if xsltproc is installed
if ! command -v xsltproc &>/dev/null; then
    echo "Error: xsltproc is not installed. Please install it using:"
    echo "sudo apt-get install xsltproc"
    rm "$XSLT_STYLESHEET" "$TEMP_URDF"
    exit 1
fi

# Apply the XSLT stylesheet to the temporary URDF file and output to stdout
xsltproc --stringparam scale_factor "$SCALE_FACTOR" --stringparam frame_prefix "$FRAME_PREFIX" "$XSLT_STYLESHEET" "$TEMP_URDF"

# Check if transformation was successful
if [ $? -ne 0 ]; then
    echo "Error: XSLT transformation failed"
    rm "$XSLT_STYLESHEET" "$TEMP_URDF"
    exit 1
fi

# Clean up the temporary files
rm "$XSLT_STYLESHEET" "$TEMP_URDF"
